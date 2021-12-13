with sim_types, dc_motor_sim, controller;
use sim_types;

with Ada.Text_IO;
use Ada.Text_IO;

procedure Controller_test is
   speed : Real;

   package Real_io is new Ada.Text_IO.Fixed_IO(Real);
   use Real_IO;
begin
   DC_motor_sim.init;

   Controller.Init(new_Kp => 0.027973811,
                   new_Ki => 0.002124903,
                   new_Kd => 0.000010000);


   Controller.Set_reference(80.0);
   -- Controller.Exec_controller_cycle;
   DC_motor_sim.Exec_cycle;

   for x in 0..2000 loop
      speed:= DC_motor_sim.Give_me_speed;
      if (x mod 10) = 0 then
         Controller.Exec_controller_cycle;
      end if;
      DC_motor_sim.Exec_cycle;
      put(speed);
      new_line;
   end loop;
   new_line;

end Controller_test;
