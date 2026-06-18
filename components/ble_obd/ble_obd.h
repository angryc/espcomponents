#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/ble_client/ble_client.h"
#include "esphome/components/esp32_ble_tracker/esp32_ble_tracker.h"
#include <esp_gattc_api.h>
#include <queue>
#include <string>
#include <vector>
#include <functional>

#ifdef USE_ESP32

namespace esphome {
namespace ble_obd {

namespace espbt = esphome::esp32_ble_tracker;

class BleObdComponent;

class BleObdDevice : public PollingComponent {
 public:
  void update() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_parent(BleObdComponent *p) { parent_ = p; }
  void set_pid(const std::string &pid) { pid_ = pid; }
  void set_mode(const std::string &mode) { mode_ = mode; }
  void set_formula(std::function<float(
    uint8_t,uint8_t,uint8_t,uint8_t,
    uint8_t,uint8_t,uint8_t,uint8_t,
    uint8_t,uint8_t,uint8_t,uint8_t,
    uint8_t,uint8_t,uint8_t,uint8_t,
    uint8_t,uint8_t,uint8_t,uint8_t,
    uint8_t,uint8_t,uint8_t,uint8_t)> f) { formula_ = f; has_formula_ = true; }
  void add_pre_command(const std::string &cmd) { pre_commands_.push_back(cmd); }

  const std::string &get_pid() const { return pid_; }
  const std::string &get_mode() const { return mode_; }
  std::string get_command() const { return mode_ + pid_; }
  const std::vector<std::string> &get_pre_commands() const { return pre_commands_; }

  virtual void on_response(const std::vector<uint8_t> &data);
  virtual void on_connect() {}
  virtual void on_disconnect() {}
  virtual void dump_config() {}

 protected:
  BleObdComponent *parent_{nullptr};
  std::string pid_;
  std::string mode_{"01"};
  std::vector<std::string> pre_commands_;
  bool has_formula_{false};
  std::function<float(
    uint8_t,uint8_t,uint8_t,uint8_t,
    uint8_t,uint8_t,uint8_t,uint8_t,
    uint8_t,uint8_t,uint8_t,uint8_t,
    uint8_t,uint8_t,uint8_t,uint8_t,
    uint8_t,uint8_t,uint8_t,uint8_t,
    uint8_t,uint8_t,uint8_t,uint8_t)> formula_;
};

class BleObdComponent : public Component, public ble_client::BLEClientNode {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::AFTER_BLUETOOTH; }

  void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                            esp_ble_gattc_cb_param_t *param) override;

  void set_service_uuid(const std::string &uuid) { service_uuid_str_ = uuid; }
  void set_rx_char_uuid(const std::string &uuid) { rx_char_uuid_str_ = uuid; }
  void set_tx_char_uuid(const std::string &uuid) { tx_char_uuid_str_ = uuid; }
  void add_init_command(const std::string &cmd) { init_commands_.push_back(cmd); }

  bool send_raw(const std::string &cmd);
  bool is_ready() const { return state_ == State::READY; }

  void add_device(BleObdDevice *d) { devices_.push_back(d); }
  void queue_device(BleObdDevice *d) { pending_.push(d); }

 protected:
  enum class State { IDLE, CONNECTED, INIT, READY };

  void process_complete_response_(const std::string &full_response);

  std::string service_uuid_str_{"FFE0"};
  std::string rx_char_uuid_str_{"FFE1"};
  std::string tx_char_uuid_str_{"FFE1"};

  espbt::ESPBTUUID service_uuid_;
  espbt::ESPBTUUID rx_char_uuid_;
  espbt::ESPBTUUID tx_char_uuid_;

  uint16_t rx_handle_{0};
  uint16_t tx_handle_{0};

  State state_{State::IDLE};
  std::vector<std::string> init_commands_;

  std::queue<std::string> tx_queue_;
  uint32_t last_tx_time_{0};
  uint32_t tx_delay_ms_{100};

  std::string response_buffer_;
  std::queue<BleObdDevice *> pending_;
  BleObdDevice *current_device_{nullptr};

  std::vector<BleObdDevice *> devices_;
};

class BleObdSensor : public sensor::Sensor, public BleObdDevice {
 public:
  void dump_config() override;
};

}  // namespace ble_obd
}  // namespace esphome

#endif
