void MoveRobot(){
  int Vx = rxStruct.Vx*max_linear_speed;
  int Vy = rxStruct.Vy*max_linear_speed;
  int W = rxStruct.W*max_angular_speed;
  float psi = rxStruct.psi;
  calc.update_angle(psi);
  calc.inverse_kinematics(Vx, Vy, W);
  float pwm_1, pwm_2, pwm_3, pwm_4 = 0;
  if(calc.Vwheel[0] > 0){
    pwm_1 = Output1 + min_motor_pwm;
  } else if (calc.Vwheel[0] < 0){
    pwm_1 = Output1 - min_motor_pwm;
  } else{
    pwm_1 = Output1;
  }

  if(calc.Vwheel[1] > 0){
    pwm_2 = Output2 + min_motor_pwm;
  } else if (calc.Vwheel[1] < 0){
    pwm_2 = Output2 - min_motor_pwm;
  } else{
    pwm_2 = Output2;
  }

  if(calc.Vwheel[2] > 0){
    pwm_3 = Output3 + min_motor_pwm;
  } else if (calc.Vwheel[2] < 0){
    pwm_3 = Output3 - min_motor_pwm;
  } else{
    pwm_3 = Output3;
  }

  if(calc.Vwheel[3] > 0){
    pwm_4 = Output4 + min_motor_pwm;
  } else if (calc.Vwheel[3] < 0){
    pwm_4 = Output4 - min_motor_pwm;
  } else{
    pwm_4 = Output4;
  }

  rangkabawah.Movement(Output1, Output2, Output3, Output4);
  //rangkabawah.Movement(calc.Vwheel[0], calc.Vwheel[1], calc.Vwheel[2], calc.Vwheel[3]);
  calc.forward_kinematics(Vfilt1, Vfilt2, Vfilt3, Vfilt4, false);
  calc.forward_kinematics(ENCFR.read(), ENCFL.read(), ENCBL.read(), ENCBR.read(), true);
}