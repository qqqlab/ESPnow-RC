#include "ESPnowRC.h"

#ifdef ESPNOWRC_DEBUG
  #define DEBUG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
  #define DEBUG_PRINTF(...) 
#endif

#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>


ESPnowRC *espnow_instance = nullptr;

//--------------------------------
// RECEIVER CALLBACKS
//--------------------------------
void ESPnowRC::rx_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status) {
  DEBUG_PRINTF("rx_send_cb:%s from=%02X:%02X:%02X:%02X:%02X:%02X\n", (status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL"), mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
}

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
void ESPnowRC::rx_recv_cb(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  uint8_t *mac_addr = info->src_addr;
#elif
void ESPnowRC::rx_recv_cb(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
#endif

  DEBUG_PRINTF("rx_recv_cb: from=%02X:%02X:%02X:%02X:%02X:%02X len=%d data=%02X %02X %02X\n", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5], len, incomingData[0], incomingData[1], incomingData[2]);
  if(!espnow_instance) return;
  if(len != sizeof(ESPnowRC::packet_t)) return;
  packet_t* pkt = (packet_t*)incomingData;

  //ignore packet if bound and sender's mac does not match
  if(espnow_instance->is_bound && memcmp(mac_addr, espnow_instance->peer_mac, 6) != 0) {
    return;
  }
  
  switch(pkt->cmd) {
    case cmd_t::BIND: //received bind packet sent to BROADCAST_MAC
    case cmd_t::CHANNEL: //received channel packet sent to RX_MAC
      //store in queue
      xQueueOverwrite(espnow_instance->xQueue, pkt->pwm);
      
      //update failsafe timestamp
      espnow_instance->failsafe_ts = millis();

      //bind peer and send BIND_REPLY
      //  pkt->cmd == cmd_t::BIND happens when tx is not bound
      //NOTE: when rx reboots then is_bound stays 0, but will process CHANNEL messages send to it's MAC
      if(pkt->cmd == cmd_t::BIND) {
        espnow_instance->bind_peer(mac_addr);
        Serial.println("send BIND_REPLY");
        pkt->cmd = cmd_t::BIND_REPLY;
        esp_now_send(mac_addr, (const uint8_t*)pkt, sizeof(packet_t) );
      }
      break;
    case cmd_t::NOT_INIT: 
    case cmd_t::BIND_REPLY:
      //do nothing
      break;
  }
}

//--------------------------------
// TRANSMITTER CALLBACKS
//--------------------------------
void ESPnowRC::tx_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status) {
  DEBUG_PRINTF("tx_send_cb:%s from=%02X:%02X:%02X:%02X:%02X:%02X\n", (status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL"), mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  if(!espnow_instance) return;
  if(status == ESP_NOW_SEND_SUCCESS) {
    espnow_instance->tx_stat.ack_cnt++;
  }
}

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
void ESPnowRC::tx_recv_cb(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  uint8_t *mac_addr = info->src_addr;
#elif
void ESPnowRC::tx_recv_cb(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
#endif

  DEBUG_PRINTF("tx_recv_cb: from=%02X:%02X:%02X:%02X:%02X:%02X len=%d data=%02X %02X %02X\n", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5], len, incomingData[0], incomingData[1], incomingData[2]);
  if(!espnow_instance) return;
  if(len != sizeof(packet_t)) return;
  packet_t* pkt = (packet_t*)incomingData;
  switch(pkt->cmd) {
    case cmd_t::BIND_REPLY:
      DEBUG_PRINTF("received BIND_REPLY\n");
      espnow_instance->bind_peer(mac_addr);
      break;
    case cmd_t::BIND:
    case cmd_t::CHANNEL:
    case cmd_t::NOT_INIT: 
      //do nothing
      break;      
  }
}

bool ESPnowRC::begin() {
  espnow_instance = this;
  is_bound = false;

  //create queue
  xQueue = xQueueCreate( 1, sizeof(queue_item_t) );

  //fill with "0" item
  //queue_item_t qitem = {};
  //xQueueOverwrite( xQueue, &qitem );

  WiFi.mode(WIFI_STA);

  //set channel
  #if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    WiFi.setChannel(channel);
  #elif
    WiFi.channel(channel);
  #endif

  // shutdown wifi
  WiFi.disconnect();
  delay(100);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESPNow Failed");
    return false;
  }

  //set Low Rate mode
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);

  //add broadcast peer
  esp_now_peer_info_t peer;
  memset(&peer, 0, sizeof(esp_now_peer_info_t));
  peer.channel = 0; // 0: use channel as set with WiFi.setChannel
  peer.encrypt = false; // can't encrypt when broadcasting
  memcpy(peer.peer_addr, BROADCAST_MAC, 6);
  esp_now_add_peer(&peer);

  //register call backs
  if(is_rx) {
    esp_now_register_send_cb(ESPnowRC::rx_send_cb);
    esp_now_register_recv_cb(ESPnowRC::rx_recv_cb);
  }else{
    esp_now_register_send_cb(ESPnowRC::tx_send_cb);
    esp_now_register_recv_cb(ESPnowRC::tx_recv_cb);   
  }

  Serial.printf("ESPNow started: mac=%s channel=%d\n", WiFi.macAddress().c_str(), (int)WiFi.channel() );

  return true;
}

bool ESPnowRC::bind_peer(const uint8_t* mac) {
  if(is_bound) return false;
  //add peer
  esp_now_peer_info_t peer;
  memset(&peer, 0, sizeof(esp_now_peer_info_t));
  peer.channel = 0; // 0: use channel as set with WiFi.setChannel
  peer.encrypt = false; // can't encrypt when broadcasting
  memcpy(peer.peer_addr, mac, 6);
  esp_now_add_peer(&peer);
  //save peer
  memcpy(peer_mac, mac, 6);
  is_bound = true;
  return true;
}

void ESPnowRC::send(uint16_t *pwm) {
  if(!pwm) return;

  //update tx_stat
  uint32_t now = micros();
  if(now - tx_stat.ts >= 1000000) {
    if(!is_bound || tx_stat.send_cnt == 0) {
      tx_linkquality = 0;
    }else{
      uint8_t lq = (uint32_t)tx_stat.ack_cnt * 100 / tx_stat.send_cnt;
      if(lq > 100) lq = 100;
      tx_linkquality = lq;
    }
    tx_stat.ack_cnt = 0;
    tx_stat.send_cnt = 0;
    tx_stat.ts = now;
  }
  tx_stat.send_cnt++;

  packet_t pkt;
  memcpy(pkt.pwm, pwm, sizeof(pkt.pwm));
  if(is_bound) {
    DEBUG_PRINTF("send: cmd=CHANNEL to=%02X:%02X:%02X:%02X:%02X:%02X\n",peer_mac[0],peer_mac[1],peer_mac[2],peer_mac[3],peer_mac[4],peer_mac[5]);
    pkt.cmd = cmd_t::CHANNEL; //channel data to RX_MAC
    esp_now_send(peer_mac, (const uint8_t*)&pkt, sizeof(pkt) );
  } else {
    DEBUG_PRINTF("send: cmd=BIND to=BROADCAST_MAC\n");
    pkt.cmd = cmd_t::BIND; //bind request + channel data to BROADCAST_MAC
    esp_now_send(BROADCAST_MAC, (const uint8_t*)&pkt, sizeof(pkt) );
  }
}

bool ESPnowRC::update(uint16_t *pwm) {
  if(!pwm) return false;
  if( xQueueReceive( xQueue, pwm, (TickType_t) 0 ) != pdPASS ) return false;
  return true;
}

bool ESPnowRC::failsafe() {
  return (millis() - failsafe_ts > failsafe_timout_ms);
}
