void MoveRobot(){
  int Vx = rxStruct.Vx*target_linear_speed;
  int Vy = rxStruct.Vy*target_linear_speed;
  int W = rxStruct.W*target_angular_speed;
  float psi = rxStruct.psi;
  calc.inverse_kin(Vx, Vy, W, psi);
  rangkabawah.Movement(Output1, Output2, Output3, Output4);
  //rangkabawah.Movement(calc.Vw[0], calc.Vw[1], calc.Vw[2], calc.Vw[3]);
  calc.forward_kin(Vr1, Vr2, Vr3, Vr4, psi);
  calc.forward_kin(ENCFR.read(), ENCFL.read(), ENCBL.read(), ENCBR.read(), psi, true);
  (abs(calc.Vr[0]) >= 1 || abs(calc.Vr[1]) >= 1 || abs(calc.Vr[2]) >= 1) ? stop = false : stop = true;
}