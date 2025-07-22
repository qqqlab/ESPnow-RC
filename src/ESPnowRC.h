#pragma once

//#define ESPNOWRC_DEBUG //uncomment to print debug info

#include <Arduino.h>
#include <esp_now.h>

class ESPnowRC {
private:
  QueueHandle_t xQueue;
  bool is_rx = false;

  struct {
    uint32_t ts = 0;
    uint16_t send_cnt = 0;
    uint16_t ack_cnt = 0;
  } tx_stat;

  uint32_t failsafe_ts = -10000; //last pwm channel packet received ms timestamp

public:
  uint8_t channel = 3; //set this before calling begin(), and set same value for both TX and RX
  uint32_t failsafe_timout_ms = 3000; //trigger failsafe after this many ms without channel data

  const uint8_t BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

  bool is_bound = false;
  uint8_t tx_linkquality = 0;
  uint8_t peer_mac[6] = {};

  enum class cmd_t : uint8_t {
    NOT_INIT    = 0x00, //not initialized
    BIND        = 0x01, //bind + channel data (from tx, always to BROADCAST_MAC)
    BIND_REPLY  = 0x02, //bind reply (from rx, always to RX_MAC)
    CHANNEL     = 0x03, //channel data (from tx, always to RX_MAC)
  };

  struct packet_t {
    cmd_t cmd;
    uint16_t pwm[16];
  } __attribute__((packed));

  struct queue_item_t {
    uint16_t pwm[16];
  };

  ESPnowRC(bool is_rx) {
    this->is_rx = is_rx;
  }

  bool begin();

  //receiver
  bool update(uint16_t *pwm); //returns true if new data received
  bool failsafe();

  //transmitter
  void send(uint16_t *pwm);

private:
  bool bind_peer(const uint8_t* mac);
  static void tx_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status);
  static void rx_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status);
  #if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    static void tx_recv_cb(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len);
    static void rx_recv_cb(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len);
  #elif
    static void tx_recv_cb(const uint8_t *mac_addr, const uint8_t *incomingData, int len);
    static void rx_recv_cb(const uint8_t *mac_addr, const uint8_t *incomingData, int len);
  #endif
};
