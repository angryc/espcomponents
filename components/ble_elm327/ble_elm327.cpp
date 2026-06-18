#include "ble_elm327.h"
#include "esphome/core/log.h"
#include <cctype>

#ifdef USE_ESP32

namespace esphome {
namespace ble_elm327 {

static const char *const TAG = "ble_elm327";
// Always sent first on connect, in this order, before any YAML init_commands.
static const char *const BASE_INIT_COMMANDS[] = {"ATZ", "ATE0", "ATL0", "ATS0", "ATH0", "ATSP0"};

static std::string normalize_command(const std::string &cmd) {
  std::string compact;
  compact.reserve(cmd.size());
  for (char c : cmd) {
    if (std::isspace(static_cast<unsigned char>(c))) continue;  // remove internal spaces too
    compact.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
  }
  return compact;
}

// ── BleElm327Device ─────────────────────────────────────────────────────────

bool BleElm327Device::on_receive(const std::vector<uint8_t> &bytes) {
  size_t skip = (pid_.size() == 2) ? 2 : 3;
  if (bytes.size() <= skip) return false;

  // Response code = 0x40 + mode (hex). Mode "01" → 0x41, mode "22" → 0x62.
  uint8_t expected_code = 0x40 + static_cast<uint8_t>(std::stoul(mode_, nullptr, 16));
  if (bytes[0] != expected_code) {
    ESP_LOGI(TAG, "Device '%s': response code mismatch: got 0x%02X, expected 0x%02X (bytes[0]=0x%02X, bytes[1]=0x%02X, bytes[2]=0x%02X, size=%zu)", 
             this->get_device_name().c_str(), bytes[0], expected_code, bytes[0], bytes.size()>1?bytes[1]:0, bytes.size()>2?bytes[2]:0, bytes.size());
    return false;
  }

  if (pid_.size() == 2) {
    if (bytes[1] != static_cast<uint8_t>(std::stoul(pid_, nullptr, 16))) {
      ESP_LOGD(TAG, "Device '%s': PID mismatch (2-char)", this->get_device_name().c_str());
      return false;
    }
  } else {
    uint8_t pid_hi = static_cast<uint8_t>(std::stoul(pid_.substr(0, 2), nullptr, 16));
    uint8_t pid_lo = static_cast<uint8_t>(std::stoul(pid_.substr(2, 2), nullptr, 16));
    if (bytes[1] != pid_hi || bytes[2] != pid_lo) {
      ESP_LOGD(TAG, "Device '%s': PID mismatch: got 0x%02X 0x%02X, expected 0x%02X 0x%02X", 
               this->get_device_name().c_str(), bytes[1], bytes[2], pid_hi, pid_lo);
      return false;
    }
  }

  std::vector<uint8_t> data(bytes.begin() + skip, bytes.end());
  ESP_LOGI(TAG, "Device '%s': matched response, data size=%zu, calling publish_data", 
           this->get_device_name().c_str(), data.size());
  publish_data(data);
  return true;
}

float BleElm327Device::parse_float(const std::vector<uint8_t> &data) {
  if (formula_.has_value()) {
    uint8_t a = data.size() > 0  ? data[0]  : 0;
    uint8_t b = data.size() > 1  ? data[1]  : 0;
    uint8_t c = data.size() > 2  ? data[2]  : 0;
    uint8_t d = data.size() > 3  ? data[3]  : 0;
    uint8_t e = data.size() > 4  ? data[4]  : 0;
    uint8_t f = data.size() > 5  ? data[5]  : 0;
    uint8_t g = data.size() > 6  ? data[6]  : 0;
    uint8_t h = data.size() > 7  ? data[7]  : 0;
    uint8_t i = data.size() > 8  ? data[8]  : 0;
    uint8_t j = data.size() > 9  ? data[9]  : 0;
    uint8_t k = data.size() > 10 ? data[10] : 0;
    uint8_t l = data.size() > 11 ? data[11] : 0;
    
    return (*formula_)(a,b,c,d,e,f,g,h,i,j,k,l);
  }
  float val = 0;
  for (size_t i = 0; i < data.size(); i++) val = val * 256.0f + data[i];
  return val;
}

void BleElm327Component::add_init_command(const std::string &cmd) {
  const std::string normalized = normalize_command(cmd);
  if (normalized.empty()) return;

  for (const char *base_cmd : BASE_INIT_COMMANDS) {
    if (normalized == normalize_command(base_cmd)) {
      ESP_LOGD(TAG, "Skip duplicate init command (already in base): %s", normalized.c_str());
      return;
    }
  }

  for (const auto &extra_cmd : extra_init_commands_) {
    if (normalized == normalize_command(extra_cmd)) {
      ESP_LOGD(TAG, "Skip duplicate init command (already queued): %s", normalized.c_str());
      return;
    }
  }

  extra_init_commands_.push_back(normalized + "\r");
}

// ── BleElm327Component ──────────────────────────────────────────────────────

void BleElm327Component::loop() {
  static bool once = false;
  if (!once) {
    ESP_LOGE(TAG, "BLE_ELM327 LOOP RUNNING");
    once = true;
  }
  if (elm_state_ == ElmState::IDLE) return;

  // Drain queued command (init or sensor) with tx_delay
  if (!tx_queue_.empty()) {
    if (millis() - last_tx_time_ < tx_delay_ms_) return;
    auto item = tx_queue_.front();
    tx_queue_.pop();
    if (item.dev) item.dev->on_dequeue();
    send_command(item.cmd);
    last_tx_time_ = millis();
    if (elm_state_ == ElmState::CONNECTED && tx_queue_.empty()) {
      elm_state_ = ElmState::READY;
      ESP_LOGD(TAG, "ELM327 initialized and ready");
    }
    return;
  }

  if (elm_state_ != ElmState::READY) return;

  // Collect one ready device per loop (round-robin to prevent starvation)
  if (!devices_.empty()) {
    size_t n = devices_.size();
    for (size_t i = 0; i < n; i++) {
      auto *d = devices_[(collect_idx_ + i) % n];
      if (d->consume_enqueued()) {
        const auto &pre = d->get_pre_commands();
        if (pre != current_pre_commands_) {
          for (const auto &c : pre) tx_queue_.push({c, nullptr});
          current_pre_commands_ = pre;
        }
        tx_queue_.push({d->get_command(), d});
        collect_idx_ = (collect_idx_ + i + 1) % n;
        break;
      }
    }
  }
}

void BleElm327Component::dump_config() {
  ESP_LOGCONFIG(TAG, "BLE ELM327:");
  ESP_LOGCONFIG(TAG, "  MAC address        : %s", this->parent_->address_str());
  char service_uuid_str[esphome::esp32_ble::UUID_STR_LEN] = {0};
  char rx_char_uuid_str[esphome::esp32_ble::UUID_STR_LEN] = {0};
  char tx_char_uuid_str[esphome::esp32_ble::UUID_STR_LEN] = {0};
  ESP_LOGCONFIG(TAG, "  Service UUID       : %s", service_uuid_.to_str(service_uuid_str));
  ESP_LOGCONFIG(TAG, "  RX Char UUID       : %s", rx_char_uuid_.to_str(rx_char_uuid_str));
  ESP_LOGCONFIG(TAG, "  TX Char UUID       : %s", tx_char_uuid_.to_str(tx_char_uuid_str));
  ESP_LOGCONFIG(TAG, "  TX delay           : %ums", tx_delay_ms_);
  ESP_LOGCONFIG(TAG, "  Base init commands : ATZ, ATE0, ATL0, ATS0, ATH0, ATSP0");
  ESP_LOGCONFIG(TAG, "  Extra init commands: %u", (unsigned)extra_init_commands_.size());
  ESP_LOGCONFIG(TAG, "  Devices            : %u", (unsigned)devices_.size());
  for (auto *d : devices_) d->dump_config();
}

void BleElm327Component::gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                                               esp_ble_gattc_cb_param_t *param) {
  ESP_LOGD(TAG, "GATTC EVENT %d", event);
  switch (event) {
    case ESP_GATTC_OPEN_EVT:
      if (param->open.status != ESP_GATT_OK) {
        ESP_LOGW(TAG, "Connection failed, status=%d", param->open.status);
        break;
      }
      gattc_if_ = gattc_if;
      memcpy(remote_bda_, param->open.remote_bda, sizeof(esp_bd_addr_t));
      client_state_ = espbt::ClientState::ESTABLISHED;
      ESP_LOGI(TAG, "Connected to ELM327");
      break;

    case ESP_GATTC_DISCONNECT_EVT:
      elm_state_ = ElmState::IDLE;
      client_state_ = espbt::ClientState::IDLE;
      rx_char_handle_ = 0;
      tx_char_handle_ = 0;
      while (!tx_queue_.empty()) tx_queue_.pop();
      for (auto *d : devices_) d->on_dequeue();
      current_pre_commands_.clear();
      response_buffer_.clear();
      response_in_progress_ = false;
      ESP_LOGW(TAG, "Disconnected from ELM327");
      break;

    case ESP_GATTC_SEARCH_CMPL_EVT: {
      auto *rx = this->parent_->get_characteristic(service_uuid_, rx_char_uuid_);
      if (rx == nullptr) {
        ESP_LOGW(TAG, "RX characteristic not found");
        break;
      }
    
      rx_char_handle_ = rx->handle;
    
      ESP_LOGI(TAG, "RX properties=0x%02X", rx->properties);
      ESP_LOGI(TAG, "RX handle=%u", rx_char_handle_);
    
      auto *tx_chr = this->parent_->get_characteristic(service_uuid_, tx_char_uuid_);
      if (tx_chr == nullptr) {
        ESP_LOGW(TAG, "TX characteristic not found");
        break;
      }
    
      tx_char_handle_ = tx_chr->handle;
    
      ESP_LOGI(TAG, "RX handle=%u TX handle=%u — registering notify",
               rx_char_handle_, tx_char_handle_);
    
      auto status = esp_ble_gattc_register_for_notify(
          gattc_if_, remote_bda_, rx_char_handle_);
    
      if (status != ESP_GATT_OK)
        ESP_LOGW(TAG, "Register notify failed: %d", status);
    
      break;
    }

    case ESP_GATTC_REG_FOR_NOTIFY_EVT:
      if (param->reg_for_notify.status != ESP_GATT_OK) {
        ESP_LOGW(TAG, "Notify registration failed: %d", param->reg_for_notify.status);
        break;
      }
      last_tx_time_ = millis();
      elm_state_ = ElmState::CONNECTED;
      for (const char *cmd : BASE_INIT_COMMANDS) {
        std::string framed = std::string(cmd) + "\r";
        tx_queue_.push({framed, nullptr});
      }
      for (const auto &cmd : extra_init_commands_) tx_queue_.push({cmd, nullptr});
       ESP_LOGD(TAG, "Queued %u init commands (%u base + %u extra)", (unsigned) tx_queue_.size(),
                (unsigned)(sizeof(BASE_INIT_COMMANDS) / sizeof(BASE_INIT_COMMANDS[0])),
                (unsigned) extra_init_commands_.size());
       break;

    case ESP_GATTC_NOTIFY_EVT: {
      ESP_LOGD(TAG,
               "NOTIFY is_notify=%d len=%u handle=%u",
               param->notify.is_notify,
               param->notify.value_len,
               param->notify.handle);
     
      std::string txt(
          (char *) param->notify.value,
          param->notify.value_len);
     
      ESP_LOGD(TAG, "RX TXT: '%s'", txt.c_str());
     
      std::string hex;
      char buf[4];
     
      for (int i = 0; i < param->notify.value_len; i++) {
        snprintf(buf, sizeof(buf), "%02X ", param->notify.value[i]);
        hex += buf;
      }
     
      ESP_LOGD(TAG, "RX HEX: %s", hex.c_str());
     
      if (param->notify.handle == rx_char_handle_)
        on_notify(param->notify.value, param->notify.value_len);
     
      break;
    } 


    case ESP_GATTC_WRITE_CHAR_EVT:
      ESP_LOGD(TAG, "WRITE OK");
      if (param->write.status != ESP_GATT_OK)
        ESP_LOGW(TAG, "Write failed, status=%d", param->write.status);
      break;

    default:
      break;
  }
}

bool BleElm327Component::send_command(const std::string &cmd) {
  if (client_state_ != espbt::ClientState::ESTABLISHED || tx_char_handle_ == 0) return false;
  auto *chr = this->parent_->get_characteristic(service_uuid_, tx_char_uuid_);
  if (chr == nullptr) { ESP_LOGW(TAG, "TX characteristic missing"); return false; }
  ESP_LOGD(TAG, "SEND HEX:");
  for (auto c : cmd)
    ESP_LOGD(TAG, "%02X", (uint8_t)c);
  ESP_LOGD(TAG, "TX handle=%u", chr->handle);
  chr->write_value(reinterpret_cast<uint8_t *>(const_cast<char *>(cmd.data())), cmd.size(),
                   ESP_GATT_WRITE_TYPE_NO_RSP);
  ESP_LOGD(TAG, ">> %s", cmd.c_str());
  ESP_LOGD(TAG, "SEND: %s", cmd.c_str());
  return true;
}

void BleElm327Component::on_notify(const uint8_t *data, uint16_t length) {
  std::string fragment(reinterpret_cast<const char *>(data), length);
  
  // Accumulate response fragments
  response_buffer_ += fragment;
  
  // Debug: show buffer state
  ESP_LOGD(TAG, "on_notify: fragment='%s', buffer='%s'", fragment.c_str(), response_buffer_.c_str());
  
  // Process all complete responses (split by '>' prompt)
  size_t pos;
  while ((pos = response_buffer_.find('>')) != std::string::npos) {
    std::string complete_response = response_buffer_.substr(0, pos + 1);  // include '>'
    response_buffer_ = response_buffer_.substr(pos + 1);
    
    ESP_LOGI(TAG, "Processing complete response: '%s'", complete_response.c_str());
    process_complete_response(complete_response);
  }
  // Remaining partial response stays in buffer
}

void BleElm327Component::process_complete_response(const std::string &full_response) {
  // Skip empty or whitespace-only responses
  if (full_response.empty() || full_response.find_first_not_of(" \t\r\n>") == std::string::npos) {
    return;
  }

  ESP_LOGD(TAG, "<< FULL RESPONSE: %s", full_response.c_str());
  ESP_LOGD(TAG, "RECV FULL: %s", full_response.c_str());

  if (elm_state_ != ElmState::READY) return;

  // Split response into lines
  std::vector<std::string> lines;
  std::string line;
  for (char c : full_response) {
    if (c == '\r' || c == '\n') {
      if (!line.empty()) {
        lines.push_back(line);
        line.clear();
      }
    } else {
      line += c;
    }
  }
  if (!line.empty()) lines.push_back(line);

  if (lines.empty()) return;

  // Debug: print all lines
  for (size_t i = 0; i < lines.size(); i++) {
    ESP_LOGD(TAG, "Line %zu: '%s'", i, lines[i].c_str());
  }

  // First line is often the command echo (e.g., "220101")
  // Skip it if it matches the last sent command (pure hex, no spaces)
  size_t start_idx = 0;
  if (!lines[0].empty() && lines[0].find_first_not_of("0123456789ABCDEFabcdef") == std::string::npos) {
    ESP_LOGD(TAG, "Skipping pure-hex command echo line 0: %s", lines[0].c_str());
    start_idx = 1;
  }

  // Skip ALL 6-hex-char lines (initial header + duplicates) before frame 0
  // These are the ELM327 3-byte header lines like "7F 22 12"
  while (start_idx < lines.size()) {
    const std::string &ln = lines[start_idx];
    // Frame headers like "0:" have format "0:..." - check for this first
    if (ln.size() >= 2 && ln[1] == ':' && isxdigit(static_cast<unsigned char>(ln[0]))) {
      break; // Found frame 0, stop skipping
    }
    // Count hex chars (ignore spaces)
    size_t hex_count = 0;
    for (char c : ln) if (isxdigit(static_cast<unsigned char>(c))) hex_count++;
    if (hex_count == 6) {
      ESP_LOGD(TAG, "Skipping ELM327 header line: %s", ln.c_str());
      start_idx++;
    } else {
      break;
    }
  }

  // Collect all hex data from remaining lines, skipping frame headers (0:, 1:, 2:, etc.)
  // and status lines (SEARCHING, CAN ERROR, NO DATA, etc.)
  // For ISO-TP multi-frame: frame 0 has response code + PID + data; frames 1+ have sequence byte + data
  std::string hex;
  int frame_num = -1;
  for (size_t i = start_idx; i < lines.size(); i++) {
    const std::string &ln = lines[i];
    
    // Skip status lines
    if (ln == "SEARCHING" || ln == "SEARCHING." || 
        ln.find("ERROR") != std::string::npos ||
        ln.find("NO DATA") != std::string::npos ||
        ln.find("UNABLE TO CONNECT") != std::string::npos ||
        ln == ">") {
      continue;
    }
    
    // Check for frame header lines like "0:", "1:", "2:", etc.
    if (ln.size() >= 2 && ln[1] == ':' && isxdigit(static_cast<unsigned char>(ln[0]))) {
      // This is a frame header line, extract hex after the colon
      frame_num = ln[0] - '0';
      std::string data_part = ln.substr(2);
      
      // For frame 0: include all data (response code + PID + data)
      // For frame 1+: skip first byte (ISO-TP sequence counter: 0x7F, 0xFF, etc.)
      bool skip_first_byte = (frame_num > 0);
      bool first_byte_done = !skip_first_byte;
      
      for (char c : data_part) {
        if (isxdigit(static_cast<unsigned char>(c))) {
          if (skip_first_byte && !first_byte_done) {
            first_byte_done = true;
            continue; // skip sequence byte
          }
          hex += c;
        }
      }
      continue;
    }
    
    // Skip lines that are just prompts or separators
    if (ln == ">" || ln == ":" || ln.empty()) continue;
    
    // Regular data line - extract hex
    for (char c : ln) {
      if (isxdigit(static_cast<unsigned char>(c))) hex += c;
    }
  }

  // Must have at least 4 hex chars (1-byte response code + 1-byte PID/data)
  if (hex.size() < 4) return;

  ESP_LOGI(TAG, "PARSED HEX: %s (len=%zu)", hex.c_str(), hex.size());

  // Parse consecutive 2-char groups into bytes
  std::vector<uint8_t> bytes;
  for (size_t i = 0; i + 1 < hex.size(); i += 2) {
    bytes.push_back(static_cast<uint8_t>(std::stoul(hex.substr(i, 2), nullptr, 16)));
  }

  if (bytes.empty()) return;

  // Debug: mode 22 with 4-char PID → skip=3 (response code + 2 PID bytes)
  constexpr size_t skip = 3;
  if (bytes.size() > skip + 11) {
    ESP_LOGI(TAG, "KONA BMS: full[4]=0x%02X full[11]=0x%02X | data[0]=0x%02X data[1]=0x%02X data[4]=0x%02X (%d) data[11]=0x%02X (%d)", 
             bytes[4], bytes[11], 
             bytes[skip + 0], bytes[skip + 1],
             bytes[skip + 4], bytes[skip + 4], 
             bytes[skip + 11], bytes[skip + 11]);
  }

  ESP_LOGI(TAG, "Broadcasting response to %zu devices", devices_.size());
  // Broadcast to all devices — each checks its own mode+PID and updates if matched
  for (auto *d : devices_) {
    ESP_LOGD(TAG, "  Trying device: '%s' mode=%s pid=%s", d->get_device_name().c_str(), d->get_mode().c_str(), d->get_pid().c_str());
    d->on_receive(bytes);
  }
}

void BleElm327Component::process_response(const std::string &response) {
  ESP_LOGD(TAG, "<< %s", response.c_str());
  ESP_LOGI(TAG, "RECV: %s", response.c_str());

  if (elm_state_ != ElmState::READY) return;

  const std::string &resp = response;

  // Strip whitespace, CR, LF, '>' — works with both ATS0 (compact) and default (spaced) format
  std::string hex;
  for (char c : resp)
    if (isxdigit(static_cast<unsigned char>(c))) hex += c;

  // Must have at least 4 hex chars (1-byte response code + 1-byte PID/data)
  if (hex.size() < 4) return;

  // Parse consecutive 2-char groups into bytes
  std::vector<uint8_t> bytes;
  for (size_t i = 0; i + 1 < hex.size(); i += 2)
    bytes.push_back(static_cast<uint8_t>(std::stoul(hex.substr(i, 2), nullptr, 16)));

  if (bytes.empty()) return;

  // Broadcast to all devices — each checks its own mode+PID and updates if matched
  for (auto *d : devices_) {
    d->on_receive(bytes);
  }
}

}  // namespace ble_elm327
}  // namespace esphome
#endif
