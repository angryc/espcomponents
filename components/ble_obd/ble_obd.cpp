#include "ble_obd.h"
#include "esphome/core/log.h"
#include <cctype>

#ifdef USE_ESP32

namespace esphome {
namespace ble_obd {

static const char *const TAG = "ble_obd";
static const char *const BASE_INIT[] = {"ATZ", "ATE0", "ATL0", "ATS0", "ATH0", "ATSP0"};

// ── BleObdComponent ──────────────────────────────────────────────────────────

void BleObdComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up BLE OBD...");
  service_uuid_ = espbt::ESPBTUUID::from_string(service_uuid_str_);
  rx_char_uuid_ = espbt::ESPBTUUID::from_string(rx_char_uuid_str_);
  tx_char_uuid_ = espbt::ESPBTUUID::from_string(tx_char_uuid_str_);
}

void BleObdComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "BLE OBD:");
  ESP_LOGCONFIG(TAG, "  Service   : %s", service_uuid_str_.c_str());
  ESP_LOGCONFIG(TAG, "  RX Char   : %s", rx_char_uuid_str_.c_str());
  ESP_LOGCONFIG(TAG, "  TX Char   : %s", tx_char_uuid_str_.c_str());
  ESP_LOGCONFIG(TAG, "  Init cmds : %u", (unsigned)init_commands_.size());
  ESP_LOGCONFIG(TAG, "  Devices   : %u", (unsigned)devices_.size());
}

void BleObdComponent::loop() {
  // Drain tx queue
  if (!tx_queue_.empty()) {
    if (millis() - last_tx_time_ < tx_delay_ms_) return;
    auto cmd = tx_queue_.front();
    tx_queue_.pop();
    this->send_raw(cmd);
    last_tx_time_ = millis();
    return;
  }

  // Queue empty + INIT state → transition to READY
  if (state_ == State::INIT && tx_queue_.empty()) {
    state_ = State::READY;
    ESP_LOGI(TAG, "ELM327 ready");
    for (auto *d : devices_) d->on_connect();
    return;
  }

  // READY + no pending → nothing to do
  if (state_ != State::READY || pending_.empty()) return;

  // Queue next pending device's commands
  auto *dev = pending_.front();
  pending_.pop();
  current_device_ = dev;

  for (const auto &cmd : dev->get_pre_commands())
    tx_queue_.push(cmd);
  tx_queue_.push(dev->get_command());
}

void BleObdComponent::gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                                           esp_ble_gattc_cb_param_t *param) {
  switch (event) {
    case ESP_GATTC_OPEN_EVT:
      if (param->open.status != ESP_GATT_OK) {
        ESP_LOGW(TAG, "Connection failed: %d", param->open.status);
        break;
      }
      ESP_LOGI(TAG, "Connected to ELM327");
      state_ = State::CONNECTED;
      break;

    case ESP_GATTC_SEARCH_CMPL_EVT: {
      ESP_LOGI(TAG, "Service search complete");
      auto *rx = this->parent()->get_characteristic(service_uuid_, rx_char_uuid_);
      if (rx == nullptr) {
        ESP_LOGW(TAG, "RX characteristic not found");
        break;
      }
      rx_handle_ = rx->handle;

      auto *tx = this->parent()->get_characteristic(service_uuid_, tx_char_uuid_);
      if (tx == nullptr) {
        ESP_LOGW(TAG, "TX characteristic not found");
        break;
      }
      tx_handle_ = tx->handle;

      esp_ble_gattc_register_for_notify(gattc_if, this->parent()->get_remote_bda(), rx_handle_);
      break;
    }

    case ESP_GATTC_REG_FOR_NOTIFY_EVT:
      if (param->reg_for_notify.status != ESP_GATT_OK) {
        ESP_LOGW(TAG, "Notify registration failed: %d", param->reg_for_notify.status);
        break;
      }
      ESP_LOGI(TAG, "Notifications registered, sending init commands");
      for (const char *cmd : BASE_INIT) tx_queue_.push(cmd);
      for (const auto &cmd : init_commands_) tx_queue_.push(cmd);
      state_ = State::INIT;
      last_tx_time_ = millis();
      break;

    case ESP_GATTC_NOTIFY_EVT: {
      if (param->notify.handle != rx_handle_) break;
      std::string fragment(reinterpret_cast<const char *>(param->notify.value),
                           param->notify.value_len);
      ESP_LOGD(TAG, "RX: '%s'", fragment.c_str());
      response_buffer_ += fragment;

      size_t pos;
      while ((pos = response_buffer_.find('>')) != std::string::npos) {
        std::string complete = response_buffer_.substr(0, pos);
        response_buffer_ = response_buffer_.substr(pos + 1);
        this->process_complete_response_(complete);
      }
      break;
    }

    case ESP_GATTC_DISCONNECT_EVT:
      ESP_LOGW(TAG, "Disconnected");
      state_ = State::IDLE;
      rx_handle_ = 0;
      tx_handle_ = 0;
      current_device_ = nullptr;
      while (!tx_queue_.empty()) tx_queue_.pop();
      while (!pending_.empty()) pending_.pop();
      response_buffer_.clear();
      for (auto *d : devices_) d->on_disconnect();
      break;

    default:
      break;
  }
}

bool BleObdComponent::send_raw(const std::string &cmd) {
  if (tx_handle_ == 0) return false;
  auto *chr = this->parent()->get_characteristic(service_uuid_, tx_char_uuid_);
  if (chr == nullptr) return false;

  std::string framed = cmd + "\r";
  chr->write_value(reinterpret_cast<const uint8_t *>(framed.data()), framed.size(),
                   ESP_GATT_WRITE_TYPE_NO_RSP);
  ESP_LOGD(TAG, ">> %s", cmd.c_str());
  return true;
}

void BleObdComponent::process_complete_response_(const std::string &full_response) {
  ESP_LOGD(TAG, "<< %s", full_response.c_str());

  std::string hex;
  for (char c : full_response)
    if (isxdigit(static_cast<unsigned char>(c))) hex += c;

  if (hex.size() < 4) return;

  std::vector<uint8_t> bytes;
  for (size_t i = 0; i + 1 < hex.size(); i += 2)
    bytes.push_back(static_cast<uint8_t>(std::stoul(hex.substr(i, 2), nullptr, 16)));

  if (bytes.empty()) return;

  // Strip response code + PID
  // Mode 01 → 0x41, skip 2 | Mode 22 → 0x62, skip 3
  uint8_t resp_code = bytes[0];
  size_t skip = (resp_code == 0x62) ? 3 : 2;

  if (bytes.size() <= skip) return;
  std::vector<uint8_t> data(bytes.begin() + skip, bytes.end());

  ESP_LOGD(TAG, "Data (%zu bytes)", data.size());

  if (current_device_) {
    current_device_->on_response(data);
  }
}

// ── BleObdSensor ─────────────────────────────────────────────────────────────

void BleObdSensor::dump_config() {
  LOG_SENSOR("  ", "BLE OBD Sensor", this);
  ESP_LOGCONFIG(TAG, "    Mode: %s  PID: %s", mode_.c_str(), pid_.c_str());
}

void BleObdSensor::queue_command() {
  if (parent_) parent_->queue_device(this);
}

void BleObdSensor::on_response(const std::vector<uint8_t> &data) {
  if (!formula_.has_value()) {
    float val = 0;
    for (uint8_t byte : data) val = val * 256.0f + byte;
    this->publish_state(val);
    return;
  }

  uint8_t a = data.size() > 0  ? data[0]  : 0;
  uint8_t b = data.size() > 1  ? data[1]  : 0;
  uint8_t c = data.size() > 2  ? data[2]  : 0;
  uint8_t d = data.size() > 3  ? data[3]  : 0;
  uint8_t e = data.size() > 4  ? data[4]  : 0;
  uint8_t f = data.size() > 5  ? data[5]  : 0;
  uint8_t g = data.size() > 6  ? data[6]  : 0;
  uint8_t h = data.size() > 7  ? data[7]  : 0;
  uint8_t ii = data.size() > 8  ? data[8]  : 0;
  uint8_t jj = data.size() > 9  ? data[9]  : 0;
  uint8_t k = data.size() > 10 ? data[10] : 0;
  uint8_t l = data.size() > 11 ? data[11] : 0;
  uint8_t m = data.size() > 12 ? data[12] : 0;
  uint8_t n = data.size() > 13 ? data[13] : 0;
  uint8_t o = data.size() > 14 ? data[14] : 0;
  uint8_t p = data.size() > 15 ? data[15] : 0;
  uint8_t q = data.size() > 16 ? data[16] : 0;
  uint8_t r = data.size() > 17 ? data[17] : 0;
  uint8_t s = data.size() > 18 ? data[18] : 0;
  uint8_t t = data.size() > 19 ? data[19] : 0;
  uint8_t u = data.size() > 20 ? data[20] : 0;
  uint8_t v = data.size() > 21 ? data[21] : 0;
  uint8_t w = data.size() > 22 ? data[22] : 0;
  uint8_t x = data.size() > 23 ? data[23] : 0;
  float val = (*formula_)(a,b,c,d,e,f,g,h,ii,jj,k,l,m,n,o,p,q,r,s,t,u,v,w,x);
  ESP_LOGI(TAG, "Sensor '%s': %.1f", this->get_name().c_str(), val);
  this->publish_state(val);
}

void BleObdSensor::on_connect() {}
void BleObdSensor::on_disconnect() {}

}  // namespace ble_obd
}  // namespace esphome

#endif
