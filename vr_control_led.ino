#include <Wire.h>
#include "VoiceRecognitionV3.h"

VR myVR(16,17);    // the (2,3) are ignored for ESP32 - kept only for easier backwards compatibility

uint8_t vr3_check_recog_cmd[]  = {0x01};
uint8_t vr3_clear_recog_cmd[]  = {0x31};
uint8_t vr3_load_records_cmd[] = {0x30, 0x00, 0x01, 0x02};
uint8_t vr3_load_response[]    = {0xAA, 0x09, 0x30, 0x03, 0x00, 0x00, 0x01, 0x00, 0x02, 0x00, 0x0A};
uint8_t vr3_stop_msg[]   = {0xAA, 0x0A, 0x0D, 0x00, 0xFF, 0x01, 0x01, 0x03, 0x4F, 0x66, 0x66, 0x0A}; // AA 0A 0D 00 FF 01 01 03 4F 66 66 0A
uint8_t vr3_run_msg[]    = {0xAA, 0x09, 0x0D, 0x00, 0xFF, 0x00, 0x00, 0x02, 0x4F, 0x6E, 0x0A}; // AA 09 0D 00 FF 00 00 02 4F 6E 0A
uint8_t vr3_buf[50];

void setup(void) {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  start_vr3_running();
}

void loop(void) {
  int ret_len;

  // Check for received voice messages from the VR3. If there's a voice message, check what it is,
  ret_len = myVR.receive_pkt(vr3_buf, 50);
  if (ret_len > 0) {
    
    if (byte_array_cmp(vr3_buf, vr3_stop_msg, ret_len, sizeof(vr3_stop_msg))) {
      Serial.println("Heard: STOP  ");
      digitalWrite(LED_BUILTIN, LOW);
    }
    else if (byte_array_cmp(vr3_buf, vr3_run_msg, ret_len, sizeof(vr3_run_msg))) {
      Serial.println("Heard: RUN   ");
      digitalWrite(LED_BUILTIN, HIGH);
    }
    else {
      Serial.println("Unknown word");
      digitalWrite(LED_BUILTIN, LOW);
    }
  }
}

//
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
