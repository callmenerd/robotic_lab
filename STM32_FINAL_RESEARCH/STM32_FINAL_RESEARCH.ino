#include "Variable.h"

// Deklarasi variabel untuk menyimpan nilai
float value1 = 0, value2 = 0, value3 = 0;

void setup(){
  Serial.begin(115200);
  pinMode(START_PUSH_BUTTON, INPUT_PULLUP);
  pinMode(RETRY_PUSH_BUTTON, INPUT_PULLUP);
  pinMode(TEAM_PUSH_BUTTON, INPUT_PULLUP);

  PID_init();
  //OBJECT CONSTRUCTING
  rangkabawah = LowerPart(sel_fr, pwm_fr, sel_fl, pwm_fl, sel_bl, pwm_bl, sel_br, pwm_br);
  calc = Kinematics(a1, a2, a3, a4, r, R);
  calc.set_ideal_value(a1_ideal, a2_ideal, a3_ideal, a4_ideal);
  calc.PPR = PPR;
  calc.init();
  myTransfer.begin(Serial);
}

void loop(){
  //Serial.println(cekwaktu);
  if (millis() - input_prevmillis >= inputrate){
    prevStateStart = start;
    prevStateRetry = retry;
    start = digitalRead(START_PUSH_BUTTON);
    retry = digitalRead(RETRY_PUSH_BUTTON);
    team = digitalRead(TEAM_PUSH_BUTTON);
    MoveRobot();
    PID_compute();
    prev_fr_tics = fr_tics; prev_fl_tics = fl_tics, prev_bl_tics = bl_tics; prev_br_tics = br_tics;
    fr_tics = ENCFR.read(); fl_tics = ENCFL.read(); bl_tics = ENCBL.read(); br_tics = ENCBR.read();
    prev_Timing = Timing;
    Timing = micros();
    Vreal1 = ((fr_tics - prev_fr_tics)/(((float)(Timing - prev_Timing))/1.0e6))/PPR*60.0;
    Vreal2 = ((fl_tics - prev_fl_tics)/(((float)(Timing - prev_Timing))/1.0e6))/PPR*60.0;
    Vreal3 = ((bl_tics - prev_bl_tics)/(((float)(Timing - prev_Timing))/1.0e6))/PPR*60.0;
    Vreal4 = ((br_tics - prev_br_tics)/(((float)(Timing - prev_Timing))/1.0e6))/PPR*60.0;
    Vfilt1 = Roda_1.updateEstimate(Vreal1);
    Vfilt2 = Roda_2.updateEstimate(Vreal2);
    Vfilt3 = Roda_3.updateEstimate(Vreal3);
    Vfilt4 = Roda_4.updateEstimate(Vreal4);

    // if (Serial.available() > 0) {
    //   String input = Serial.readStringUntil('\n'); // Membaca input sampai newline
    //   input.trim(); // Menghapus spasi di awal/akhir input

    //   if (input.indexOf(' ') == -1) { // Jika tidak ada spasi, berarti input tunggal
    //     float singleValue = input.toFloat();
    //     if(singleValue == 1.0){
    //       rxStruct.Vx = 0;
    //       rxStruct.Vy = 1;
    //       rxStruct.W = 0;
    //     } else if(singleValue == 2.0){
    //       rxStruct.Vx = 0;
    //       rxStruct.Vy = 0.7;
    //       rxStruct.W = 0;
    //     } else if(singleValue == 3.0){
    //       rxStruct.Vx = 0;
    //       rxStruct.Vy = 0.4;
    //       rxStruct.W = 0;
    //     } else if(singleValue == 4.0){
    //       rxStruct.Vx = 0;
    //       rxStruct.Vy = 0;
    //       rxStruct.W = 0;
    //     }
    //   } else { // Jika ada spasi, berarti input terdiri dari beberapa nilai
    //     int firstSpace = input.indexOf(' ');
    //     int secondSpace = input.indexOf(' ', firstSpace + 1);

    //     value1 = input.substring(0, firstSpace).toFloat();
    //     value2 = input.substring(firstSpace + 1, secondSpace).toFloat();
    //     value3 = input.substring(secondSpace + 1).toFloat();

    //     motor1.SetTunings(value1, value2, value3);
    //     motor2.SetTunings(value1, value2, value3);
    //     motor3.SetTunings(value1, value2, value3);
    //     motor4.SetTunings(value1, value2, value3);
    //   }
    // }
    // Serial.print("Vxset:");
    // Serial.print(rxStruct.Vx*100);
    // Serial.print(",");
    // Serial.print("Vyset:");
    // Serial.print(rxStruct.Vy*100);
    // Serial.print(",");
    // Serial.print("Wset:");
    // Serial.print(rxStruct.W*100);
    // Serial.print(",");
    // Serial.print("Vx:");
    // Serial.print(calc.Vreal[0]);
    // Serial.print(",");
    // Serial.print("Vy:");
    // Serial.print(calc.Vreal[1]);
    // Serial.print(",");
    // Serial.print("W:");
    // Serial.print(calc.Vreal[2]);
    // Serial.println("");
    // Serial.print("Vset1:");
    // Serial.print(calc.Vwheel[0]);
    // Serial.print(",");
    // Serial.print("Vset2:");
    // Serial.print(calc.Vwheel[1]);
    // Serial.print(",");
    // Serial.print("Vset3:");
    // Serial.print(calc.Vwheel[2]);
    // Serial.print(",");
    // Serial.print("Vset4:");
    // Serial.print(calc.Vwheel[3]);
    // Serial.print(",");
    // Serial.print("Vreal1:");
    // Serial.print(Vfilt1);
    // Serial.print(",");
    // Serial.print("Vreal2:");
    // Serial.print(Vfilt2);
    // Serial.print(",");
    // Serial.print("Vreal3:");
    // Serial.print(Vfilt3);
    // Serial.print(",");
    // Serial.print("Vreal4:");
    // Serial.print(Vfilt4);
    // Serial.println("");
    txStruct.timestamp = (float) (Timing-prev_Timing);
    txStruct.X = (float) calc.dist_travel[0];
    txStruct.Y = (float) calc.dist_travel[1];
    txStruct.tetha = (float) calc.dist_travel[2];
    txStruct.Vx = (float) calc.Vreal[0];
    txStruct.Vy = (float) calc.Vreal[1];
    txStruct.Wr = (float) calc.Vreal[2];
    txStruct.team = (char) team;
    if(!start){
      if(prevStateStart == true){
        ENCFR.readAndReset();
        ENCFL.readAndReset();
        ENCBL.readAndReset();
        ENCBR.readAndReset();
      }
      txStruct.cmd = 'M';}
    else if(!retry){
      if(prevStateRetry == true){
        ENCFR.readAndReset();
        ENCFL.readAndReset();
        ENCBL.readAndReset();
        ENCBR.readAndReset();
      }
      txStruct.cmd = 'R';}
    else{
      txStruct.cmd = 'S';
      if(!prevStateStart || !prevStateRetry){
        ENCFR.readAndReset();
        ENCFL.readAndReset();
        ENCBL.readAndReset();
        ENCBR.readAndReset();
      }
      calc.dist_travel[0] = 0;
      calc.dist_travel[1] = 0;
      calc.dist_travel[2] = 0;
      rxStruct.Vx = 0;
      rxStruct.Vy = 0;
      rxStruct.W = 0;
      txStruct.Vx = 0.0;
      txStruct.Vy = 0.0;
      txStruct.Wr = 0.0;
    }
    if(myTransfer.available()){
      uint16_t sendSize = 0;
      sendSize = myTransfer.txObj(txStruct, sendSize);
      myTransfer.sendData(sendSize);

      uint16_t recSize = 0;
      recSize = myTransfer.rxObj(rxStruct, recSize);
    }
    input_prevmillis = millis();
  }
}