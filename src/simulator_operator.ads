with sim_types;
use sim_types;

package Simulator_operator is
   procedure Carry_out_a_simulation(Kp, Ki, Kd: in Real; Score: out Real);
end Simulator_operator;
