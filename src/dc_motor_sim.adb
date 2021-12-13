
package body DC_motor_sim is

   motor_T: Real;
   w_ec_w_factor, w_ec_i_factor : Real;
   i_ec_v_factor, i_ec_i_factor : Real;

   R, L, J, fv, K, ea: Real;
   i, w: array(0..1) of Real;


   procedure init is
   begin
      motor_T := 0.0001;
      R := 1.11; -- Ohm;
      L := 0.0002; -- H
      J := 6.77E-6; -- Kgm²
      fv := 1.66E-5; -- Nm/(Rad/s)
      K := 0.0364; -- Nm/A or V/(Rad/s)
      ea := 0.0;
      w(0) := 0.0; -- tk angular speed
      w(1) := 0.0; -- t(k+1) angular speed
      i(0) := 0.0; -- tk current
      i(1) := 0.0; -- t(k+1) current

      w_ec_w_factor := Real(1.0) - Real(motor_T*fv)/Real(J);
      w_ec_i_factor := Real(motor_T*K)/Real(J);

      i_ec_v_factor := Real(motor_T)/Real(L);
      i_ec_i_factor := Real(1.0) - Real(motor_T*R)/Real(L);
   end init;

   procedure Set_ea(new_ea: in Real) is
   begin
      ea := new_ea;
      -- Saturation effect due to battery limits
      if ea > 24.0 then
         ea := 24.0;
      end if;
      if ea < -24.0 then
         ea := -24.0;
      end if;
   end Set_ea;

   function Give_me_speed return Real is
   begin
      return w(0);
   end Give_me_speed;

   procedure Exec_cycle is
   begin
      -- Motor behavior considering T = 0.0001s;
      w(0) := w(1);
      i(0) := i(1);
      w(1) := w_ec_w_factor*w(0) + w_ec_i_factor*i(0);
      i(1) := i_ec_i_factor*i(0) + i_ec_v_factor*(ea - K*w(0));
   end Exec_cycle;
end DC_motor_sim;
