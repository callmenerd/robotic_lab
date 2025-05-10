/* ---- LIBRARY --- */
#include <Wire.h>
#include <SpeedTrig.h>
#include <SimpleKalmanFilter.h>
#include <QuickPID.h>
#include "Encoder.h"
#include "SpeedTrig.h"
#include "LowerPart.h"
#include "Kinematics.h"
#include "SerialTransfer.h"

/* --- TIMINGS --- */
unsigned long input_prevmillis = 0;
uint8_t inputrate = 5; /*4 ms for 1,25 m/s 
                      10 ms for 0,5 m/s
                      20 ms for 0.25 m/s
                      to get uncertainty with 0,5 cm periode of control system
                      use this formula = (min_measurement (cm)*1000)/velocity (cm/s)
                      */

/* --- OBJECT --- */
LowerPart rangkabawah;
Kinematics calc;

/* --- DC MOTOR for movement  PIN --- */
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

/* --- CMD PUSH BUTTON --- */
#define TEAM_PUSH_BUTTON PB10
#define START_PUSH_BUTTON PA1
#define RETRY_PUSH_BUTTON PA15
bool start, retry, team = false;

/* --- QUADRATURE ENCODER MOTOR --- */
const float PPR = 537.6; //PPR Encoder
Encoder_internal_state_t * Encoder::interruptArgs[];
Encoder ENCFR(enc_fr_a, enc_fr_b, PPR);
Encoder ENCFL(enc_fl_a, enc_fl_b, PPR);
Encoder ENCBL(enc_bl_a, enc_bl_b, PPR);
Encoder ENCBR(enc_br_a, enc_br_b, PPR);

/* --- SERIAL COM --- */
SerialTransfer myTransfer;