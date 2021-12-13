with sim_types;
use sim_types;

package Controller is

   procedure Init(new_Kp, new_Ki, new_Kd: in Real);

   procedure Set_reference(Angular_speed: in Real);

   procedure Exec_controller_cycle;

private
   Kp, Ki, Kd: Real;
   Angular_speed_reference: Real;
   E: array(-1..0) of Real;
   Ei, Ed : Real; -- Errors
end Controller;
