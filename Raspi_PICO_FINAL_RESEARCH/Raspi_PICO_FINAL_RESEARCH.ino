#include "Header.h"

void setup() {
  Serial.begin(115200);
  filter.begin();
  // Begin SerialTransfer
  myTransfer.begin(Serial);
}

void setup1(){
  Wire.setSDA(0);
  Wire.setSCL(1);

  // Initialize the sensors.
  BMX.begin();
  BMX.getAllDataCalibrated();
}

void loop(void) {
  if(millis()-input_prev >= input_rate){
    filter.updateIMU(gx, gy, gz, ax, ay, az);
    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw()-180;
    txStruct.timestamp = (float) (cekwaktu - prevcekwaktu);
    txStruct.roll = roll-savedgx; txStruct.pitch = pitch-savedgy; txStruct.yaw = heading-savedgz;
    txStruct.gx = gx; txStruct.gy = gy; txStruct.gz = gz;
    txStruct.ax = acc[0]; txStruct.ay = acc[1]; txStruct.az = az;

    // Send data
    txStruct.roll = filter.getRoll()-savedgx; txStruct.pitch = filter.getPitch()-savedgy; txStruct.yaw =  filter.getYaw()-180-savedgz;
    if(myTransfer.available()){
      uint16_t sendSize = 0;
      sendSize = myTransfer.txObj(txStruct, sendSize);
      myTransfer.sendData(sendSize);
      uint16_t recSize = 0;
      recSize = myTransfer.rxObj(rxStruct, recSize);
    }
    input_prev = millis();
    // Serial.print(roll);
    // Serial.print(",");
    // Serial.print(pitch);
    // Serial.print(",");
    // Serial.print(heading);
    // Serial.println();
  
    // Cetak hasil estimasi untuk monitoring
    // Serial.print("Time:"); Serial.print(txStruct.timestamp); Serial.print(",");
    // Serial.print("Real_ax:"); Serial.print(ax); Serial.print(",");
    // Serial.print("Real_ay:"); Serial.print(ay); Serial.print(",");
    // Serial.print("Estimated_ax:"); Serial.print(acc[0]); Serial.print(",");
    // Serial.print("Estimated_ay:"); Serial.println(acc[1]);
  }
  //Handle received data from Raspberry Pi
  prevMecState = mechanism_state;
  machine_state = rxStruct.command;
  mechanism_state = rxStruct.mechanism_state;
  if (machine_state == 'M' || machine_state == 'R'){

  } else{
    // Save last gyro values if stop signal is first received
    savedgx = filter.getRoll();
    savedgy = filter.getPitch();
    savedgz = filter.getYaw()-180;
  }
}

void loop1(){
  if(micros()-sensor_prev >= sensor_rate){
    BMX.getAllDataCalibrated();
    // gx = drift_elim(BMX.gyro[0], prevgx, 0);
    // gy = drift_elim(BMX.gyro[1], prevgy, 1);
    // gz = drift_elim(BMX.gyro[2], prevgz, 2);

    gx = BMX.gyro[0];
    gy = BMX.gyro[1];
    gz = BMX.gyro[2];

    ax = BMX.accel[0];
    ay = BMX.accel[1];
    az = BMX.accel[2];
    
    acc[0] = ax_calc.updateEstimate(ax);
    acc[1] = ay_calc.updateEstimate(ay);

    prevcekwaktu = cekwaktu;
    cekwaktu = micros();
    sensor_prev = micros();
  }
}