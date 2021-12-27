with DC_motor_sim, Controller;


package body Simulator_operator is

   procedure Carry_out_a_simulation(Kp, Ki, Kd: in Real; Score: out Real) is

      speed: array(1..2000) of Real; -- Samples during 0.2s
      abort_simulation : boolean := false;
      val_reg_perm_reached : boolean := false;
      Tp_checked : boolean := false;
      val_reg_perm : Real;
      val : Real;
      percentage : Natural;
      Tp : Integer;
      Tr, Ts : integer;
      --Tp_value
      Mp : Real;
      Controller_reference : constant Real := 80.0;
      Expected_Tr : constant integer := 140; -- /14 ms
      -- Expected_Tp : constant integer := 140; -- /14 ms
      Expected_Mp : constant Real := 0.07*Controller_reference; -- 7%
      -- Expected_Ts : constant integer := 900; --/10 ms
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
         val_reg_perm := speed(2000);
         Tr := 0;

         -- We penalize oscilations only before the value on permament regimen is reached. We also penalize Tr bigger than 15ms or lower than 13ms
         for x in 2..2000 loop
            if not val_reg_perm_reached then
               if speed(x) > val_reg_perm then
                  val_reg_perm_reached := True;
                  if (x > 140) or (x < 130) then
                     Tr := integer'last/4;
                  else
                     Tr := x;
                  end if;
               end if;
               if speed(x) < speed(x-1) then
                  Tr := integer'last/4;
                  exit;
               end if;
            end if;

            --Tp management
            if val_reg_perm_reached and not Tp_checked then
               Tp_checked := True;
               if speed(x) <= speed(x-1) then
                  val := speed(x) - val_reg_perm;
                  if val = Expected_Mp then
                     Mp := val;
                  else
                     if val > Expected_Mp then
                        percentage := Natural(val*100/speed(x));
                        Mp := Real(float(val) ** (percentage - 6));
                     else
                        Mp := Real'Last/2.0;
                     end if;
                  end if;
               end if;
            end if;
         end loop;

         -- A better score is a lower score
         Score := Real(abs(Tr - Expected_Tr)) + Real(abs(Mp - Expected_Mp));
      end if;

   end Carry_out_a_simulation;

end Simulator_operator;
