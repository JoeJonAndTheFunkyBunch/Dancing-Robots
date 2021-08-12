
//this is the basic code to make sure two of the dancing robots(rats) work together with the esp-now and spiffs libraries 
//P.S you need to download all the included libraries to your arduino IDE for everything to work smoothly
#include "AiEsp32RotaryEncoder.h"
#include "Arduino.h"
#include "FS.h"
#include "SPIFFS.h"
#define FORMAT_SPIFFS_IF_FAILED true
#include <ESP32Servo.h>
#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include "AiEsp32RotaryEncoder.h"
#include "Arduino.h"
#include "FS.h"
#include "SPIFFS.h"
Servo servos[5];

#define ROTARY_ENCODER_BUTTON_PIN 90
#define ROTARY_ENCODER_VCC_PIN -1 /* 27 put -1 of Rotary encoder Vcc is connected directly to 3,3V; else you can use declared output pin for powering rotary encoder */

#define ROTARY_ENCODER_ACCERLATION  20

#define ROTARY_ENCODER_MIN 0
#define ROTARY_ENCODER_MAX 30

#define MIN_PULSE_WIDTH_MG90S 1000 // small servo
#define MAX_PULSE_WIDTH_MG90S 2000

#define MIN_PULSE_WIDTH_MG996R 600 //big servo
#define MAX_PULSE_WIDTH_MG996R 2500

#define SOUND_IO_0_PIN  25

#define ROTARY_ENCODER_STEPS 4 //depending on your encoder - try 1,2 or 4 to get expected behaviour
#define NUM_SERVOS  5
// Recommended PWM GPIO pins on the ESP32 include 2,4,12-19,21-23,25-27,32-33
byte servo_pins[] = {32, 27, 13, 12, 14};

byte servo_start_deg[] = {84, 72, 90, 90, 90};
byte servo_min_deg[] = {0, 30, 24, 16, 0};
byte servo_max_deg[] = {180, 126, 180, 180, 180};

int servo_min_pulse_width[] = {MIN_PULSE_WIDTH_MG996R, MIN_PULSE_WIDTH_MG996R, MIN_PULSE_WIDTH_MG90S, MIN_PULSE_WIDTH_MG90S, MIN_PULSE_WIDTH_MG90S};
int servo_max_pulse_width[] = {MAX_PULSE_WIDTH_MG996R, MAX_PULSE_WIDTH_MG996R, MAX_PULSE_WIDTH_MG90S, MAX_PULSE_WIDTH_MG90S, MAX_PULSE_WIDTH_MG90S};

byte rotary_encoder_a_pin[] = {23, 21, 18, 4, 26};
byte rotary_encoder_b_pin[] = {22, 19, 5, 15, 25};
byte rotary_encoder_btn_pin[] = { ROTARY_ENCODER_BUTTON_PIN,
                                  ROTARY_ENCODER_BUTTON_PIN,
                                  ROTARY_ENCODER_BUTTON_PIN,
                                  ROTARY_ENCODER_BUTTON_PIN,
                                  ROTARY_ENCODER_BUTTON_PIN
                                };
bool recording = false;
long startTime;
int lastPosition[NUM_SERVOS];

typedef struct
{
  long ms;
  int servoIndex;
  int value;
}  dance_move_type;

int currentMove = 0;
dance_move_type dance_moves[1000];
//EspNow
uint8_t broadcastAddress[] = {0x24,0x6F,0x28,0xA9,0x80,0xB8};
//variables to store incoming readings
int pan;
int tilt;
int rh;
int lh;
int head;
//variables to store outcoming reading
int incomingPan;
int incomingTilt;
int incomingRh;
int incomingLh;
int incomingHead;

typedef struct struct_message {
  int PanDt;
  int TiltDt;
  int RhDt;
  int LhDt;
  int HeadDt;
}struct_message;
String success;
//structure message to hold incoming readings
struct_message ServoDt;
//structure message to send outcoming readings
struct_message incomingServoDt;
//instead of changing here, rather change numbers above
AiEsp32RotaryEncoder rotaryEncoder_pan, rotaryEncoder_tilt, rotaryEncoder_lh, rotaryEncoder_rh, rotaryEncoder_head;
// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nlast Packet Send Status:\t");
  Serial.print(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if(status == 0) {
    success = "Delivery Success";
  }else{
    success = "Delivery Fail";
  }
}
//callback when data is sent
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingServoDt, incomingData, sizeof(incomingServoDt));
  Serial.print("bytes received");
  Serial.println(len);
  incomingTilt = incomingServoDt.TiltDt;
  incomingPan = incomingServoDt.PanDt;
  incomingRh = incomingServoDt.RhDt;
  incomingLh = incomingServoDt.LhDt;
  incomingHead = incomingServoDt.HeadDt;
}
int EspNowData[] = {rotaryEncoder_pan.readEncoder(), rotaryEncoder_tilt.readEncoder(), rotaryEncoder_lh.readEncoder(), rotaryEncoder_rh.readEncoder(),rotaryEncoder_head.readEncoder()};
//Serial.print(EspNowData[0] + "Test")
void rotary_onButtonClick()
{
  static unsigned long lastTimePressed = 0;
  //ignore multiple press in that time milliseconds
  if (millis() - lastTimePressed < 500)
  {
    return;
  }
  lastTimePressed = millis();
  Serial.print("button pressed at ");
  Serial.println(millis());
}

void rotary_loop()
{

  //dont print anything unless value changed
  if (!rotaryEncoder_pan.encoderChanged() && !rotaryEncoder_tilt.encoderChanged() && !rotaryEncoder_rh.encoderChanged()  && !rotaryEncoder_lh.encoderChanged() && !rotaryEncoder_head.encoderChanged() )
  {
    return;
  }
//  int EspNowData[] = {rotaryEncoder_pan.readEncoder()};
  if(EspNowData[0] && EspNowData[1] && EspNowData[2] && EspNowData[3] && EspNowData[4] != NULL) {
  Serial.print(EspNowData[0] + "Test");
  Serial.println(EspNowData[1]+"Test");
  Serial.println(EspNowData[2]+ "Test");
  Serial.println(EspNowData[3]+ "Test");
  Serial.println(EspNowData[4]+"Test");
}else{
  return;
}
  int readCurrent[] = { rotaryEncoder_pan.readEncoder(),
                        rotaryEncoder_tilt.readEncoder(),
                        rotaryEncoder_rh.readEncoder(),
                        rotaryEncoder_lh.readEncoder(),
                        rotaryEncoder_head.readEncoder()
                      };

  Serial.print(readCurrent[0]);
  Serial.print(" ");
  Serial.print(readCurrent[1]);
  Serial.print(" ");
  Serial.print(readCurrent[2]);
  Serial.print(" ");
  Serial.print(readCurrent[3]);
  Serial.print(" ");
  Serial.println(readCurrent[4]);

  for (int i = 0; i < NUM_SERVOS ; i++) {
    if (lastPosition[i] != readCurrent[i]) {
      moveServoToPosition(i, readCurrent[i]);
      if (recording) {
        dance_moves[currentMove].ms = millis() - startTime;
        dance_moves[currentMove].servoIndex = i;
        dance_moves[currentMove].value = readCurrent[i];
        currentMove++;
        dance_moves[currentMove].ms = -1; //mark end of recording
      }
    }
    lastPosition[i] = readCurrent[i];
  }


}

void setup()
{
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);

  pinMode(SOUND_IO_0_PIN, OUTPUT);
  digitalWrite(SOUND_IO_0_PIN, HIGH);

  pinMode(ROTARY_ENCODER_BUTTON_PIN, INPUT);
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_send_cb(OnDataSent);
  //register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress,6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  
  //add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("failed to add peer");
    return; 
  }
    esp_now_register_recv_cb(OnDataRecv);
  
  dance_moves[0].ms = -1; //mark end of recording

  if (!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)) {
    Serial.println("SPIFFS Mount Failed");
  }

  bool circleValues = false;

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  for (int a = 0; a < NUM_SERVOS ; a++) {
    servos[a].setPeriodHertz(50);    // standard 50 hz servo
    servos[a].attach(servo_pins[a], servo_min_pulse_width[a], servo_max_pulse_width[a]);
    //    servos[a].attach(servo_pins[a], MIN_PULSE_WIDTH_MG996R, 50
  }

  int i = 0;
  rotaryEncoder_pan = AiEsp32RotaryEncoder(rotary_encoder_a_pin[i], rotary_encoder_b_pin[i], rotary_encoder_btn_pin[i], ROTARY_ENCODER_VCC_PIN, ROTARY_ENCODER_STEPS);
  rotaryEncoder_pan.begin();
  rotaryEncoder_pan.setup(
    [] { rotaryEncoder_pan.readEncoder_ISR(); },
    [] { rotary_onButtonClick(); });
  rotaryEncoder_pan.setBoundaries(ROTARY_ENCODER_MIN, ROTARY_ENCODER_MAX, circleValues); //minValue, maxValue, circleValues true|false (when max go to min and vice versa)
  rotaryEncoder_pan.reset(map(servo_start_deg[i], servo_min_deg[i], servo_max_deg[i], ROTARY_ENCODER_MIN, ROTARY_ENCODER_MAX));


  i = 1;
  rotaryEncoder_tilt = AiEsp32RotaryEncoder(rotary_encoder_a_pin[i], rotary_encoder_b_pin[i], rotary_encoder_btn_pin[i], ROTARY_ENCODER_VCC_PIN, ROTARY_ENCODER_STEPS);
  rotaryEncoder_tilt.begin();
  rotaryEncoder_tilt.setup(
    [] { rotaryEncoder_tilt.readEncoder_ISR(); },
    [] { rotary_onButtonClick(); });
  rotaryEncoder_tilt.setBoundaries(ROTARY_ENCODER_MIN, ROTARY_ENCODER_MAX, circleValues); //minValue, maxValue, circleValues true|false (when max go to min and vice versa)
  rotaryEncoder_tilt.reset(map(servo_start_deg[i], servo_min_deg[i], servo_max_deg[i], ROTARY_ENCODER_MIN, ROTARY_ENCODER_MAX));

  i = 2;
  rotaryEncoder_rh = AiEsp32RotaryEncoder(rotary_encoder_a_pin[i], rotary_encoder_b_pin[i], rotary_encoder_btn_pin[i], ROTARY_ENCODER_VCC_PIN, ROTARY_ENCODER_STEPS);
  rotaryEncoder_rh.begin();
  rotaryEncoder_rh.setup(
    [] { rotaryEncoder_rh.readEncoder_ISR(); },
    [] { rotary_onButtonClick(); });
  rotaryEncoder_rh.setBoundaries(ROTARY_ENCODER_MIN, ROTARY_ENCODER_MAX, circleValues); //minValue, maxValue, circleValues true|false (when max go to min and vice versa)
  rotaryEncoder_rh.reset(map(servo_start_deg[i], servo_min_deg[i], servo_max_deg[i], ROTARY_ENCODER_MIN, ROTARY_ENCODER_MAX));

  i = 3;
  rotaryEncoder_lh = AiEsp32RotaryEncoder(rotary_encoder_a_pin[i], rotary_encoder_b_pin[i], rotary_encoder_btn_pin[i], ROTARY_ENCODER_VCC_PIN, ROTARY_ENCODER_STEPS);
  rotaryEncoder_lh.begin();
  rotaryEncoder_lh.setup(
    [] { rotaryEncoder_lh.readEncoder_ISR(); },
    [] { rotary_onButtonClick(); });
  rotaryEncoder_lh.setBoundaries(ROTARY_ENCODER_MIN, ROTARY_ENCODER_MAX, circleValues); //minValue, maxValue, circleValues true|false (when max go to min and vice versa)
  rotaryEncoder_lh.reset(map(servo_start_deg[i], servo_min_deg[i], servo_max_deg[i], ROTARY_ENCODER_MIN, ROTARY_ENCODER_MAX));

  i = 4;
  rotaryEncoder_head = AiEsp32RotaryEncoder(rotary_encoder_a_pin[i], rotary_encoder_b_pin[i], rotary_encoder_btn_pin[i], ROTARY_ENCODER_VCC_PIN, ROTARY_ENCODER_STEPS);
  rotaryEncoder_head.begin();
  rotaryEncoder_head.setup(
    [] { rotaryEncoder_head.readEncoder_ISR(); },
    [] { rotary_onButtonClick(); });
  rotaryEncoder_head.setBoundaries(ROTARY_ENCODER_MIN, ROTARY_ENCODER_MAX, circleValues); //minValue, maxValue, circleValues true|false (when max go to min and vice versa)
  rotaryEncoder_head.reset(map(servo_start_deg[i], servo_min_deg[i], servo_max_deg[i], ROTARY_ENCODER_MIN, ROTARY_ENCODER_MAX));

  /*Rotary acceleration introduced 25.2.2021.
       in case range to select is huge, for example - select a value between 0 and 1000 and we want 785
       without accelerateion you need long time to get to that number
       Using acceleration, faster you turn, faster will the value raise.
       For fine tuning slow down.
  */
  //rotaryEncoder.disableAcceleration(); //acceleration is now enabled by default - disable if you dont need it
  rotaryEncoder_pan.setAcceleration(ROTARY_ENCODER_ACCERLATION); //or set the value - larger number = more accelearation; 0 or 1 means disabled acceleration
  rotaryEncoder_tilt.setAcceleration(ROTARY_ENCODER_ACCERLATION); //or set the value - larger number = more accelearation; 0 or 1 means disabled acceleration
  rotaryEncoder_rh.setAcceleration(ROTARY_ENCODER_ACCERLATION); //or set the value - larger number = more accelearation; 0 or 1 means disabled acceleration
  rotaryEncoder_lh.setAcceleration(ROTARY_ENCODER_ACCERLATION); //or set the value - larger number = more accelearation; 0 or 1 means disabled acceleration
  rotaryEncoder_head.setAcceleration(ROTARY_ENCODER_ACCERLATION); //or set the value - larger number = more accelearation; 0 or 1 means disabled acceleration


  for (int i = 0 ; i < NUM_SERVOS ; i++) {
    servos[i].write(servo_start_deg[i]);
  }

}

void loop()
{
   ServoDt.PanDt = pan;
   ServoDt.TiltDt = tilt;
   ServoDt.RhDt = rh;
   ServoDt.LhDt = lh;
   ServoDt.HeadDt = head;

   //send message throught esp-now
   esp_err_t result = esp_now_send(broadcastAddress,(uint8_t*) &ServoDt, sizeof(ServoDt));

   if(result == ESP_OK) {
    Serial.println("Sent with success");
   }else {
    Serial.println("Error sending the data");
   }
   
  if (Serial.available())
  {
    char command = Serial.read();

    if (command == 'r') {
      Serial.println("recording");
      digitalWrite(SOUND_IO_0_PIN, LOW);
      recording = true;
      currentMove = 0;
      startTime = millis();
    }

    if (command == 's') {
      Serial.println("stop");
      digitalWrite(SOUND_IO_0_PIN, HIGH);
      recording = false;
    }

    if (command == 'p') {
      Serial.println("play");
      digitalWrite(SOUND_IO_0_PIN, LOW);
      startTime = millis();
      currentMove = 0;
      bool interrupt = false;
      while (dance_moves[currentMove].ms > -1 && interrupt == false) {
        if (millis() - startTime > dance_moves[currentMove].ms) {
          moveServoToPosition(dance_moves[currentMove].servoIndex, dance_moves[currentMove].value);
          currentMove++;
        }
        if (Serial.available()) {
          char c = Serial.read();
          if (c == 's') interrupt = true;
        }

      }
      digitalWrite(SOUND_IO_0_PIN, HIGH);

    }

  }

  rotary_loop();
  // delay(5);
}

void moveServoToPosition(int servoIndex, int pos) {

  pos = map(pos, ROTARY_ENCODER_MIN, ROTARY_ENCODER_MAX, servo_min_deg[servoIndex], servo_max_deg[servoIndex]);
  servos[servoIndex].write(pos);

  Serial.print("Moving servo ");
  Serial.print(servoIndex);
  Serial.print(" to ");
  Serial.println(pos);

}

String readFileToStr(fs::FS &fs, const char * path) {
  Serial.printf("Reading file into string: %s\r\n", path);

  File file = fs.open(path);
  if (!file || file.isDirectory()) {
    Serial.println("- failed to open file for reading");
    return "error";
  }


  String str = "";
  while (file.available()) {
    str = str + char(file.read());
  }

  Serial.println(str);

  return str;
}
