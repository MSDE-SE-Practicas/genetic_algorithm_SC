with sim_types;
use sim_types;

package DC_motor_sim is


      procedure init;

      procedure Set_ea(new_ea: in Real);

      function Give_me_speed return Real;

      procedure Exec_cycle;

end DC_motor_sim;
