void PID_init(){
  motor1.SetOutputLimits(-250, 250);
  motor2.SetOutputLimits(-250, 250);
  motor3.SetOutputLimits(-250, 250);
  motor4.SetOutputLimits(-250, 250);

  motor1.SetSampleTimeUs(10000);
  motor2.SetSampleTimeUs(10000);
  motor3.SetSampleTimeUs(10000);
  motor4.SetSampleTimeUs(10000);

  motor1.SetTunings(Kp1, Ki1, Kd1);
  motor2.SetTunings(Kp2, Ki2, Kd2);
  motor3.SetTunings(Kp3, Ki3, Kd3);
  motor4.SetTunings(Kp4, Ki4, Kd4);

  motor1.SetMode(motor1.Control::automatic);
  motor2.SetMode(motor2.Control::automatic);
  motor3.SetMode(motor3.Control::automatic);
  motor4.SetMode(motor4.Control::automatic);
}

void PID_compute(){
  motor1.Compute();
  motor2.Compute();
  motor3.Compute();
  motor4.Compute();
}

void PID_reset(){
  motor1.Reset();
  motor2.Reset();
  motor3.Reset();
  motor4.Reset();
}