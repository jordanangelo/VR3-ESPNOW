
#include <Wire.h>
#include "VoiceRecognitionV3.h"
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h> // only for esp_wifi_set_channel()

// Global copy of slave
esp_now_peer_info_t slave;
#define CHANNEL 1
#define PRINTSCANRESULTS 0
#define DELETEBEFOREPAIR 0

VR myVR(16,17);    // the (2,3) are ignored for ESP32 - kept only for easier backwards compatibility

uint8_t vr3_check_recog_cmd[]  = {0x01};
uint8_t vr3_clear_recog_cmd[]  = {0x31};
uint8_t vr3_load_records_cmd[] = {0x30, 0x00, 0x01, 0x02};
uint8_t vr3_load_response[]    = {0xAA, 0x09, 0x30, 0x03, 0x00, 0x00, 0x01, 0x00, 0x02, 0x00, 0x0A};
uint8_t vr3_stop_msg[]   = {0xAA, 0x0A, 0x0D, 0x00, 0xFF, 0x01, 0x01, 0x03, 0x4F, 0x66, 0x66, 0x0A}; // AA 0A 0D 00 FF 01 01 03 4F 66 66 0A
uint8_t vr3_run_msg[]    = {0xAA, 0x09, 0x0D, 0x00, 0xFF, 0x00, 0x00, 0x02, 0x4F, 0x6E, 0x0A}; // AA 09 0D 00 FF 00 00 02 4F 6E 0A
uint8_t vr3_buf[50];

// Init ESP Now with fallback
void InitESPNow() {
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  }
  else {
    Serial.println("ESPNow Init Failed");
    // Retry InitESPNow, add a counte and then restart?
    // InitESPNow();
    // or Simply Restart
    ESP.restart();
  }
}

// Scan for slaves in AP mode
void ScanForSlave() {
  int16_t scanResults = WiFi.scanNetworks(false, false, false, 300, CHANNEL); // Scan only on one channel
  // reset on each scan
  bool slaveFound = 0;
  memset(&slave, 0, sizeof(slave));

  Serial.println("");
  if (scanResults == 0) {
    Serial.println("No WiFi devices in AP Mode found");
  } else {
    Serial.print("Found "); Serial.print(scanResults); Serial.println(" devices ");
    for (int i = 0; i < scanResults; ++i) {
      // Print SSID and RSSI for each device found
      String SSID = WiFi.SSID(i);
      int32_t RSSI = WiFi.RSSI(i);
      String BSSIDstr = WiFi.BSSIDstr(i);

      if (PRINTSCANRESULTS) {
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(SSID);
        Serial.print(" (");
        Serial.print(RSSI);
        Serial.print(")");
        Serial.println("");
      }
      delay(10);
      // Check if the current device starts with `Slave`
      if (SSID.indexOf("Slave") == 0) {
        // SSID of interest
        Serial.println("Found a Slave.");
        Serial.print(i + 1); Serial.print(": "); Serial.print(SSID); Serial.print(" ["); Serial.print(BSSIDstr); Serial.print("]"); Serial.print(" ("); Serial.print(RSSI); Serial.print(")"); Serial.println("");
        // Get BSSID => Mac Address of the Slave
        int mac[6];
        if ( 6 == sscanf(BSSIDstr.c_str(), "%x:%x:%x:%x:%x:%x",  &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5] ) ) {
          for (int ii = 0; ii < 6; ++ii ) {
            slave.peer_addr[ii] = (uint8_t) mac[ii];
          }
        }

        slave.channel = CHANNEL; // pick a channel
        slave.encrypt = 0; // no encryption

        slaveFound = 1;
        // we are planning to have only one slave in this example;
        // Hence, break after we find one, to be a bit efficient
        break;
      }
    }
  }

  if (slaveFound) {
    Serial.println("Slave Found, processing..");
  } else {
    Serial.println("Slave Not Found, trying again.");
  }

  // clean up ram
  WiFi.scanDelete();
}

// Check if the slave is already paired with the master.
// If not, pair the slave with master
bool manageSlave() {
  if (slave.channel == CHANNEL) {
    if (DELETEBEFOREPAIR) {
      deletePeer();
    }

    Serial.print("Slave Status: ");
    // check if the peer exists
    bool exists = esp_now_is_peer_exist(slave.peer_addr);
    if ( exists) {
      // Slave already paired.
      Serial.println("Already Paired");
      return true;
    } else {
      // Slave not paired, attempt pair
      esp_err_t addStatus = esp_now_add_peer(&slave);
      if (addStatus == ESP_OK) {
        // Pair success
        Serial.println("Pair success");
        return true;
      } else if (addStatus == ESP_ERR_ESPNOW_NOT_INIT) {
        // How did we get so far!!
        Serial.println("ESPNOW Not Init");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_ARG) {
        Serial.println("Invalid Argument");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_FULL) {
        Serial.println("Peer list full");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_NO_MEM) {
        Serial.println("Out of memory");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_EXIST) {
        Serial.println("Peer Exists");
        return true;
      } else {
        Serial.println("Not sure what happened");
        return false;
      }
    }
  } else {
    // No slave found to process
    Serial.println("No Slave found to process");
    return false;
  }
}

void deletePeer() {
  esp_err_t delStatus = esp_now_del_peer(slave.peer_addr);
  Serial.print("Slave Delete Status: ");
  if (delStatus == ESP_OK) {
    // Delete success
    Serial.println("Success");
  } else if (delStatus == ESP_ERR_ESPNOW_NOT_INIT) {
    // How did we get so far!!
    Serial.println("ESPNOW Not Init");
  } else if (delStatus == ESP_ERR_ESPNOW_ARG) {
    Serial.println("Invalid Argument");
  } else if (delStatus == ESP_ERR_ESPNOW_NOT_FOUND) {
    Serial.println("Peer not found.");
  } else {
    Serial.println("Not sure what happened");
  }
}

uint8_t data = 0;
// send data
void sendData() {
  int ret_len;
  // Check for received voice messages from the VR3. If there's a voice message, check what it is,
  ret_len = myVR.receive_pkt(vr3_buf, 50);
  if (ret_len > 0) {
    if (byte_array_cmp(vr3_buf, vr3_stop_msg, ret_len, sizeof(vr3_stop_msg))) {
      Serial.println("Heard: STOP  ");
      data = 0;
      const uint8_t *peer_addr = slave.peer_addr;
      Serial.print("Sending: "); Serial.println(data);
      esp_err_t result = esp_now_send(peer_addr, &data, sizeof(data));
      Serial.print("Send Status: ");
      if (result == ESP_OK) {
        Serial.println("Success");
      } else if (result == ESP_ERR_ESPNOW_NOT_INIT) {
        Serial.println("ESPNOW not Init.");
      } else if (result == ESP_ERR_ESPNOW_ARG) {
        Serial.println("Invalid Argument");
      } else if (result == ESP_ERR_ESPNOW_INTERNAL) {
        Serial.println("Internal Error");
      } else if (result == ESP_ERR_ESPNOW_NO_MEM) {
        Serial.println("ESP_ERR_ESPNOW_NO_MEM");
      } else if (result == ESP_ERR_ESPNOW_NOT_FOUND) {
        Serial.println("Peer not found.");
      } else {
        Serial.println("Not sure what happened");
      }
    }
    else if (byte_array_cmp(vr3_buf, vr3_run_msg, ret_len, sizeof(vr3_run_msg))) {
      Serial.println("Heard: RUN   ");
      data = 1;
      const uint8_t *peer_addr = slave.peer_addr;
      Serial.print("Sending: "); Serial.println(data);
      esp_err_t result = esp_now_send(peer_addr, &data, sizeof(data));
      Serial.print("Send Status: ");
      if (result == ESP_OK) {
        Serial.println("Success");
      } else if (result == ESP_ERR_ESPNOW_NOT_INIT) {
        Serial.println("ESPNOW not Init.");
      } else if (result == ESP_ERR_ESPNOW_ARG) {
        Serial.println("Invalid Argument");
      } else if (result == ESP_ERR_ESPNOW_INTERNAL) {
        Serial.println("Internal Error");
      } else if (result == ESP_ERR_ESPNOW_NO_MEM) {
        Serial.println("ESP_ERR_ESPNOW_NO_MEM");
      } else if (result == ESP_ERR_ESPNOW_NOT_FOUND) {
        Serial.println("Peer not found.");
      } else {
        Serial.println("Not sure what happened");
      }
    }
    else {
      Serial.println("Unknown word");
    }
  }
}

// callback when data is sent from Master to Slave
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Sent to: "); Serial.println(macStr);
  Serial.print("Last Packet Send Status: "); Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  Serial.begin(115200);
  start_vr3_running();
  //Set device in STA mode to begin with
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE);
  Serial.println("ESPNow/Basic/Master Example");
  // This is the mac address of the Master in Station Mode
  Serial.print("STA MAC: "); Serial.println(WiFi.macAddress());
  Serial.print("STA CHANNEL "); Serial.println(WiFi.channel());
  // Init ESPNow with a fallback logic
  InitESPNow();
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
}

void loop() {
  int ret_len;
  // Check for received voice messages from the VR3. If there's a voice message, check what it is,
  ret_len = myVR.receive_pkt(vr3_buf, 50);

  // In the loop we scan for slave
  ScanForSlave();
  // If Slave is found, it would be populate in `slave` variable
  // We will check if `slave` is defined and then we proceed further
  if (slave.channel == CHANNEL) { // check if slave channel is defined
    // `slave` is defined
    // Add slave as peer if it has not been added already
    bool isPaired = manageSlave();
    if (isPaired) {
      sendData();
    } else {
      // slave pair failed
      Serial.println("Slave pair failed!");
    }
  }
  else {
    // No slave found to process
  }
}

// Starts the VR3 module running, by loading the records into the recognizer and reporting the
// result on the OLED.
void start_vr3_running(void) {
  myVR.begin(9600);     // 9600 baud serial port between ESP32 and VR3

  // Check the recognizer and clear the recognizer. Get the resulting responses but ignore them, we don't
  // care, it's only to get the VR3 started in a known clear state
  myVR.receive_pkt(vr3_buf, 50);    // start with a read in case there's any junk data in the receive fifo
  myVR.send_pkt(vr3_check_recog_cmd, sizeof(vr3_check_recog_cmd));
  myVR.receive_pkt(vr3_buf, 50);
  myVR.send_pkt(vr3_clear_recog_cmd, sizeof(vr3_clear_recog_cmd));
  myVR.receive_pkt(vr3_buf, 50);
  
  // Now tell the VR3 recognizer to load the word recognition records
  // Check the response to ensure the VR3 did load the recognizer correctly.
  myVR.send_pkt(vr3_load_records_cmd, sizeof(vr3_load_records_cmd));
  
  if (check_for_vr3_load_response(50))
    Serial.println("Listening...");
  else
    Serial.println("VR3 Not Started");  
}

boolean check_for_vr3_load_response(int timeout) {
  int ret_len;
  
  ret_len = myVR.receive_pkt(vr3_buf, timeout);
  if (ret_len <= 0)
    return false;

  if (byte_array_cmp(vr3_buf, vr3_load_response, ret_len, sizeof(vr3_load_response)))
    return true;
   
  return false;
}

boolean byte_array_cmp(uint8_t *a, uint8_t *b, int len_a, int len_b) {
      int n;
      // if their lengths are different, return false
      if (len_a != len_b) return false;

      // test each element to be the same. if not, return false
      for (n=0;n<len_a;n++) if (a[n]!=b[n]) return false;

      //ok, if we have not returned yet, they are equal :)
      return true;
}
