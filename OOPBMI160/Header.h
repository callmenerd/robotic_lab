#include <Servo.h>
#include "SerialTransfer.h"
#include "BMI160_lib.h"

#define speed_up 190
#define speed_down 60
#define KANAN_RPWM 7
#define KANAN_LPWM 6
#define KIRI_RPWM 9
#define KIRI_LPWM 8
#define servokanan 5
#define servokiri 10

SerialTransfer myTransfer;
BMI160_lib BMI;

Servo right_servo;
Servo left_servo;

int moveTime = 50; // Move every 0.5s
unsigned long int startMillis = 0;
int step = 1; // Move 5 degrees
int position = 0;
char a;

unsigned long input_prevmillis = 0;
byte inputrate = 50;

int limit_up = 11;
int limit_down = 12;

int limit_atas, prev_limit_atas = 0;
int limit_bawah, prev_limit_bawah = 0;
bool naik, turun = false;

bool close, open = false;

struct __attribute__((packed)) STRUCTRX {
  float Vx;
  float Vy;
  float W;
  char command;
} rxStruct;

struct __attribute__((packed)) STRUCTTX {
  float Vrx;
  float Vry;
  float Wr;
  char feedback_respon;
} txStruct;

char rec[4];