#include "Header.h"
void setup()
{
  Wire.setSDA(0);
  Wire.setSCL(1);
  Wire.begin();
  Wire.setClock(400000); //400khz clock
  Serial.begin(115200);
  myTransfer.begin(Serial);
  BMI.setup();
  left_servo.attach(10); //kiri
  left_servo.write(120);
  right_servo.attach(5); //kanan
  right_servo.write(60);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(KANAN_RPWM, OUTPUT);
  pinMode(KANAN_LPWM, OUTPUT);
  pinMode(KIRI_RPWM, OUTPUT);
  pinMode(KIRI_LPWM, OUTPUT);
  pinMode(limit_up, INPUT_PULLUP);
  pinMode(limit_down, INPUT_PULLUP);
}

void loop()
{
  BMI.updateValue();
  // convert aerospace quaternion to aerospace Euler
  // Serial.print(BMI.roll);
  // Serial.print(" | ");
  // Serial.print(BMI.pitch);
  // Serial.print(" | ");
  // Serial.print(BMI.yaw);
  // Serial.print(" | ");
  // if(Serial.available()){
  //   a = Serial.read();
  // }
  // if(a == 'a'){
  //   open = true;
  //   close = false;
  // }  
  // if(a == 'b'){
  //   close = true;
  //   open = false;
  // }

  // if(a == 'c'){
  //   naik = true;
  // }  
  // if(a == 'd'){
  //   turun = true;
  // }

  if(millis()-startMillis >= moveTime){
    startMillis = millis();
    if(open){
      left_servo.write(80);
      right_servo.write(100);
    }
    if(close){
      left_servo.write(175);
      right_servo.write(5);
    }
  }

  prev_limit_atas = limit_atas;
  prev_limit_bawah = limit_bawah;
  limit_atas = digitalRead(limit_up);
  limit_bawah = digitalRead(limit_down);
  while(naik == true){
    limit_atas = digitalRead(limit_up);
    limit_bawah = digitalRead(limit_down);
    up();
    if(limit_atas == 0) naik = false;
  }
  while(turun == true){
    limit_atas = digitalRead(limit_up);
    limit_bawah = digitalRead(limit_down);
    down();
    if(limit_bawah == 0) turun = false;
  }
  stop();
  // Serial.print(left_servo.read());
  // Serial.print(" | ");
  // Serial.print(right_servo.read());
  // Serial.print(" | ");
  // Serial.print(naik);
  // Serial.print(" | ");
  // Serial.println(turun);
  txStruct.Vrx = BMI.roll;
  txStruct.Vry = BMI.pitch;
  txStruct.Wr = BMI.yaw;
  txStruct.feedback_respon = rxStruct.command;
  if(myTransfer.available()){
    uint16_t sendSize = 0;
    sendSize = myTransfer.txObj(txStruct, sendSize);
    myTransfer.sendData(sendSize);
    uint16_t recSize = 0;
    recSize = myTransfer.rxObj(rxStruct, recSize);
  }
  (rxStruct.command == '1') ? digitalWrite(LED_BUILTIN, HIGH):digitalWrite(LED_BUILTIN, LOW);
}

void up(){
      analogWrite(KANAN_RPWM, speed_up);
      analogWrite(KANAN_LPWM, 0);

      analogWrite(KIRI_RPWM, speed_up);
      analogWrite(KIRI_LPWM, 0);
}
void down(){
      analogWrite(KANAN_RPWM, 0);
      analogWrite(KANAN_LPWM, speed_down);

      analogWrite(KIRI_RPWM, 0);
      analogWrite(KIRI_LPWM, speed_down);
}
void stop(){
      analogWrite(KANAN_RPWM, 0);
      analogWrite(KANAN_LPWM, 0);

      analogWrite(KIRI_RPWM, 0);
      analogWrite(KIRI_LPWM, 0);
}
