/* ---- LIBRARY --- */
#include <Wire.h>
#include <SpeedTrig.h>
#include <SimpleKalmanFilter.h>
#include <QuickPID.h>
#include "Encoder.h"
#include "SpeedTrig.h"
#include "LowerPart.h"
#include "Kinematics.h"

/* ---- DC MOTOR for movement  PIN ---- */
//-- Kanan depan
#define sel_fr PA4
#define pwm_fr PB6
#define enc_fr_b PA8
#define enc_fr_a PB14
//-- Kiri depan
#define sel_fl PA5
#define pwm_fl PB9
#define enc_fl_b PA9
#define enc_fl_a PB15
//-- Kiri belakang
#define sel_bl PA2
#define pwm_bl PB8
#define enc_bl_b PA6
#define enc_bl_a PB12
//-- Kanan belakang
#define sel_br PA3
#define pwm_br PB7
#define enc_br_b PA7
#define enc_br_a PB13
// //-- Kanan depan
// #define sel_fr PA2
// #define pwm_fr PB8
// #define enc_fr_b PA6
// #define enc_fr_a PB12
// //-- Kiri depan
// #define sel_fl PA3
// #define pwm_fl PB7
// #define enc_fl_b PA7
// #define enc_fl_a PB13
// //-- Kiri belakang
// #define sel_bl PA4
// #define pwm_bl PB6
// #define enc_bl_b PA8
// #define enc_bl_a PB14
// //-- Kanan belakang
// #define sel_br PA5
// #define pwm_br PB9
// #define enc_br_b PA9
// #define enc_br_a PB15

#define enc_mr_a PB0
#define enc_mr_b PB1
#define enc_ml_a PB3
#define enc_ml_b PB4

/* --- CMD --- */
#define TEAM_PUSH_BUTTON PB10
#define START_PUSH_BUTTON PA1
#define RETRY_PUSH_BUTTON PA15

// #define LimitR PB0
// #define LimitL PB1

bool start, retry, team = false;
//Ultrasonik
volatile float duration_kanan, duration_kiri = 0;
float distance_kanan, distance_kiri = 0;

/* --- TIMINGS --- */
//-- Timing input joystick
unsigned long input_prevmillis = 0;
byte inputrate = 10;

/* QUADRATURE ENCODER MOTOR) */
Encoder_internal_state_t * Encoder::interruptArgs[];
Encoder ENCFR(enc_fr_a, enc_fr_b);
Encoder ENCFL(enc_fl_a, enc_fl_b);
Encoder ENCBL(enc_bl_a, enc_bl_b);
Encoder ENCBR(enc_br_a, enc_br_b);