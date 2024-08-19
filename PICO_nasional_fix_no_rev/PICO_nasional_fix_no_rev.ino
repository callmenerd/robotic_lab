#include "Header.h"
#include "Lifter.h"

Lifter lifter;

unsigned long long ultrasonic_prevL;
unsigned long long delay_prev;
unsigned long long ultrasonic_prevM;
unsigned long long ultrasonic_prevR;
float h, s, v;
char send_response, a;
byte prev_in, prev_out = 0;

void setup() {
  Serial.begin(115200);

  // Setup Lifter
  lifter = Lifter(SLIDER_RPWM, SLIDER_LPWM, ROLLER_INTAKE_RPWM, ROLLER_INTAKE_LPWM);

  // Setup sensor and serial
  sensor_setup();
  // Begin SerialTransfer
  //myTransfer.begin(Serial);
}

void loop() {
  if(millis() - prevT_imu >=2.5){
    BMX.getAllDataCalibrated();
    gx = drift_elim(BMX.gyro[0], prevgx, 0);
    gy = drift_elim(BMX.gyro[1], prevgy, 1);
    gz = drift_elim(BMX.gyro[2], prevgz, 2);
    prevgx = BMX.gyro[0];
    prevgy = BMX.gyro[1];
    prevgz = BMX.gyro[2];
    filter.updateIMU(gx, gy, gz, BMX.accel[0], BMX.accel[1], BMX.accel[2]);
    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw()-180; 
    prevT_imu = millis();
  }  

  // Send data
  prevState = rxStruct.command;
  prevMecState = rxStruct.mechanism_state;
  txStruct.roll = filter.getRoll();-savedgx;
  txStruct.pitch = filter.getPitch();-savedgy;
  txStruct.yaw =  filter.getYaw()-180-savedgz;
  if(Serial.available()){
    a = Serial.read();
    Serial.print("You typed: " );
    Serial.println(a);
  }
  if(a == 'a'){
    take_ball = true;
    put_ball = false;
  } else if(a == 'b'){
    take_ball = false;
    put_ball = true;
  } else if(a == 'c'){
    take_ball = false;
    put_ball = false;
  }
  // if(myTransfer.available()){
  //   uint16_t sendSize = 0;
  //   sendSize = myTransfer.txObj(txStruct, sendSize);
  //   myTransfer.sendData(sendSize);
  //   uint16_t recSize = 0;
  //   recSize = myTransfer.rxObj(rxStruct, recSize);
  // }

  // Handle received data from Raspberry Pi
  char machine_state = rxStruct.command;
  char mechanism_state = rxStruct.mechanism_state;
  (mechanism_state != 'N') ? txStruct.feedback_respon = send_response:txStruct.feedback_respon = 'U';
  if (machine_state == 'M' || machine_state == 'R'){
    if(prevMecState != mechanism_state){
      if (mechanism_state == 'T'){
        take_ball = true;
        put_ball = false;
      } else if(mechanism_state == 'P'){
        put_ball = true;
        take_ball = false;
      } else if(mechanism_state == 'N'){
        put_ball = false;
        take_ball = false;
      }
    }
  } else{
    // Save last gyro values if stop signal is first received
    savedgx = filter.getRoll();
    savedgy = filter.getPitch();
    savedgz = filter.getYaw()-180;
  }
  mechanism_act();

  //Read Ultrasonic
}

void mechanism_act(){
  int in = digitalRead(IR_CHECKIN);
  int out = digitalRead(IR_CHECKOUT);
  if (take_ball){
    if(in == 0){
      Serial.print(in);
    Serial.print(" | ");
    Serial.println(out);
      if(prev_in == 1){
        lifter.system_off();
        read_color();
      } else{
        lifter.do_outtake();
        lifter.do_up_lift();
      }
    } else if(prev_in == 0 && in == 1){
      if(h>355 || h<=30){
        send_response = 'm';
      } else if(h>270 && h<=355){
        send_response = 'p';
      } else if(h>=200 && h<=270){
        send_response = 'b';
      }
      lifter.system_off();
      Serial.println(send_response);
      take_ball = false;
    } else{
      lifter.intake_and_lift();
    }
  } 
  if (put_ball) {
    if(out == 1){
      if (prev_out == 0){
        lifter.lift_off();
        put_ball = false;
      }
    } else{
      lifter.do_up_lift();
    }
  }
  prev_in = in;
  prev_out = out;
}

void read_color(){
  digitalWrite(color_led, HIGH);
  float red, green, blue;
  volatile long delaying = millis();
  while(millis()-delaying<=120){
    ;
  }
  tcs.getRGB(&red, &green, &blue);
  RGBtoHSV(red, green, blue, h, s, v);
  digitalWrite(color_led, LOW);
}

float drift_elim(float raw, float prevraw, int idx){
  float mean = 0;
  if(abs(raw) <= 0.005){
    if(abs(raw)*driftCompensate >= abs(prevraw)){
      if(countUP >= sampling){
        countUP = 0;
        mean = gmean[idx]/sampling;
        gmean[idx] = 0;
      } else{
        countDN = 0;
        countUP++;
        gmean[idx] += raw;
      }
    } else if(abs(raw)*driftCompensate <= abs(prevraw)){
      if(countDN >= sampling){
        countDN = 0;
        mean = gmean[idx]/sampling;
        gmean[idx] = 0;
      } else{
        countUP = 0;
        countDN++;
        gmean[idx] += raw;
      }
    } else{
      countUP = 0;
      countDN = 0;
      gmean[idx] = 0;
    }
  } else{
    countUP = 0;
    countDN = 0;
    mean = raw;
  }
  return mean;
}