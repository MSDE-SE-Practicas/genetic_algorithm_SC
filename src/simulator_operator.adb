with DC_motor_sim, Controller;


package body Simulator_operator is

   procedure Carry_out_a_simulation(Kp, Ki, Kd: in Real; Score: out Real) is

      speed: array(1..2000) of Real; -- Samples during 0.2s
      abort_simulation : boolean := false;
      --Tp : integer;
      Tr, Ts : integer;
      --Tp_value, Mp : Real;
      Controller_reference : constant Real := 80.0;
      Expected_Tr : constant integer := 120; -- /12 ms
      Expected_Tp : constant integer := 140; -- /14 ms
      Expected_Mp : constant Real := 0.05*Controller_reference; -- 5%
      Expected_Ts : constant integer := 900; --/10 ms
   begin
      -- Init motor simulator and speed controller
      Dc_motor_sim.init;
      Controller.Init(Kp, Ki, Kd);

      -- Set controller reference
      Controller.Set_reference(Controller_reference);

      -- Init speed
      for x in 1..2000 loop
         speed(x) := 0.0;
      end loop;

      -- Simulate

      Controller.Exec_controller_cycle;
      DC_motor_sim.Exec_cycle;

      for x in 1..2000 loop
         speed(x) := DC_motor_sim.Give_me_speed;

         -- Verify extrem values
         if (speed(x) > Controller_reference*5.0) or (speed(x) < 0.0) then
            abort_simulation := true;
            exit; -- Abort simulation
         end if;

         if ((x mod 10) = 0) then
            -- The controller runs every milisecond
            Controller.Exec_controller_cycle;
         end if;
         DC_motor_sim.Exec_cycle;
      end loop;

      -- Evaluate results
      if abort_simulation then
         Score := Real'Last/2.0;
      else
         -- Look for rise time
         Tr := 0;
         -- In this case we penalize oscillations
         for x in 2..2000 loop
            if speed(x) < speed(x-1) then
               Tr := integer'last/4;
               exit;
            end if;
         end loop;

         if Tr = 0 then
            for x in reverse 1..2000 loop
               if speed(x) > Controller_reference*0.63 then
                  Tr := x;
               end if;
            end loop;
         end if;

         -- Look for settling time
         if (Tr > 0) and (Tr < 2000) then
            for x in Tr..2000 loop
               if abs(Controller_reference - speed(x)) > Controller_reference*0.02 then
                  Ts := x;
               end if;
            end loop;
         else
            Ts := integer'Last/4;
         end if;

         -- A better score is a lower score
         Score := Real(abs(Tr - Expected_Tr)) + Real(abs(Ts - Expected_Ts));
      end if;

   end Carry_out_a_simulation;

end Simulator_operator;
