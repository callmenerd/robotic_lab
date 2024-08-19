// Libraries
#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <Madgwick_BMX160.h>
#include "CalibrateBMX160.h"
#include "SerialTransfer.h"

// set to false if using a common cathode LED
#define commonAnode true

#define SLIDER_RPWM 6 
#define SLIDER_LPWM 7
#define ROLLER_INTAKE_RPWM 10
#define ROLLER_INTAKE_LPWM 11
#define IR_CHECKIN 17
#define IR_CHECKOUT 16
#define ECHO_L 20
#define TRIG_L 21
#define TRIG_M 27
#define ECHO_M 26
#define PING_R 18
#define color_led 29

// SerialTransfer
SerialTransfer myTransfer;
CalibrateBMX160 BMX;
//Mahony_BMX160 filter;
Madgwick_BMX160 filter;

char prevState;
char prevMecState;

struct __attribute__((packed)) STRUCTRX {
  char mechanism_state;
  char command;
} rxStruct;

struct __attribute__((packed)) STRUCTTX {
  float roll;
  float pitch;
  float yaw;
  char feedback_respon;
  float dist_R;
  float dist_L;
  float dist_M;
} txStruct;

//Timing
int moveTime = 50; // Move every 0.5s
unsigned long int startMillis = 0;
int step = 1; // Move 5 degrees
int position = 0;

unsigned long input_prevmillis = 0;
byte inputrate = 50;

//Variabel
bool take_ball = false;
bool put_ball = false;
char prev_machine_state = ' ';
char prev_mechanism_state = ' ';

float savedgx, savedgy, savedgz = 0.0;

const float driftCompensate = 0.995;
const int sampling = 16;
int countUP, countDN = 0;
float gx, gy, gz;
float prevgx, prevgy, prevgz = 0;
float init_x, init_y, init_z = 0;
float gmean[3] = {0,0,0};
float roll, pitch, heading = 0;
unsigned long prevT = 0;

// our RGB -> eye-recognized gamma color
byte gammatable[256];

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

unsigned long prev_T = 0;
const int input_rate = 10;

// Variables for the ultrasonic sensor tasks
float dist_L, dist_R, dist_M;
unsigned long prevT_imu = 0;
const unsigned long input_imu = 2.5; // Sensor update interval in ms
unsigned long prevT_sensor = 0;
const unsigned long input_sensor = 25; // Sensor update interval in ms
// Variables to store the duration and distance
volatile long duration1, distance1;
volatile long duration2, distance2;
volatile long duration3, distance3;

// Timing variables
unsigned long previousMillis1 = 0;
unsigned long previousMillis2 = 0;
unsigned long previousMillis3 = 0;
const long interval = 50; // 50ms interval for sensor readings

byte color = 0;

// Function to convert RGB to HSV
void RGBtoHSV(float r, float g, float b, float &h, float &s, float &v) {
  float rd = (float)r / 255;
  float gd = (float)g / 255;
  float bd = (float)b / 255;
  
  float max_val = max(rd, max(gd, bd));
  float min_val = min(rd, min(gd, bd));
  
  v = max_val; // value is the maximum of r, g, b
  
  float delta = max_val - min_val;
  if (max_val != 0)
    s = delta / max_val; // saturation
  else {
    s = 0;
    h = -1;
    return;
  }
  
  if (rd == max_val)
    h = (gd - bd) / delta; // hue between yellow & magenta
  else if (gd == max_val)
    h = 2 + (bd - rd) / delta; // hue between cyan & yellow
  else
    h = 4 + (rd - gd) / delta; // hue between magenta & cyan
  
  h *= 60; // convert to degrees
  if (h < 0) h += 360;
}

void echo1ISR() {
  if (digitalRead(ECHO_L) == HIGH) {
    duration1 = micros();
  } else {
    duration1 = micros() - duration1;
    distance1 = duration1 * 0.034 / 2;
  }
}

void echo2ISR() {
  if (digitalRead(ECHO_M) == HIGH) {
    duration2 = micros();
  } else {
    duration2 = micros() - duration2;
    distance2 = duration2 * 0.034 / 2;
  }
}

// For the single pin sensor, a simple timing method can be used.
void measureSinglePinSensor() {
  duration3 = pulseIn(PING_R, HIGH);
  distance3 = duration3 * 0.034 / 2;
}

void sensor_setup() {
  pinMode(IR_CHECKIN, INPUT);
  pinMode(IR_CHECKOUT, INPUT);
  pinMode(ECHO_L, INPUT);
  pinMode(TRIG_L, OUTPUT);
  pinMode(ECHO_M, INPUT);
  pinMode(TRIG_M, OUTPUT);
  pinMode(color_led, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ECHO_L), echo1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ECHO_M), echo2ISR, CHANGE);
  // Begin I2C
  Wire.setSDA(0);
  Wire.setSCL(1);
  Wire.requestFrom(0x68, 1);
  Wire.begin();
  Wire1.setSDA(2);
  Wire1.setSCL(3);
  Wire1.requestFrom(0x29, 1);
  Wire1.begin();

  BMX.begin();
  BMX.getAllDataCalibrated();
  tcs.begin(0x29, &Wire1);
  for (int i=0; i<256; i++) {
    float x = i;
    x /= 255;
    x = pow(x, 2.5);
    x *= 255;

    if (commonAnode) {
      gammatable[i] = 255 - x;
    } else {
      gammatable[i] = x;
    }
  }
  filter.begin();
}

void triggerSensor(int trigPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
}

void triggerSinglePinSensor(int pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  delayMicroseconds(2);
  digitalWrite(pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(pin, LOW);
  pinMode(pin, INPUT);
}