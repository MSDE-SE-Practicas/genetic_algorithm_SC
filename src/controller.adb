with DC_motor_sim;

package body Controller is

   procedure Init(new_Kp, new_Ki, new_Kd: in Real) is
   begin
      Kp := new_Kp;
      Ki := new_Ki;
      Kd := new_kd;

      -- Init controller variables of error
      E(-1) := 0.0;
      E(0) := 0.0;
      Ei := 0.0;
      Ed := 0.0;
   end Init;

   procedure Set_reference(Angular_speed: in Real) is
   begin
      Angular_speed_reference := Angular_speed;
   end Set_reference;

   procedure Exec_controller_cycle is
      motor_speed: Real;
      new_motor_voltage : Real;
      Controller_T: Real := 0.001;
   begin
      -- Compute speed error
      motor_speed := DC_motor_sim.Give_me_speed;
      E(0) := Angular_speed_reference - motor_speed;
      Ei := Ei + E(0);
      Ed := (E(0) - E(-1))/Controller_T;

      -- Prepare last error for next cycle
      E(-1) := E(0);

      -- Compute controller output
      new_motor_voltage := Kp*E(0) + Ki*Ei + Kd*Ed;
      DC_motor_sim.Set_ea(new_motor_voltage);

   end Exec_controller_cycle;

end Controller;
