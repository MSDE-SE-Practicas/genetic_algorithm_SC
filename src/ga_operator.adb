with sim_types;
use sim_types;

with Ada.Numerics.Float_Random, Ada.Text_IO;
use Ada.Numerics.Float_Random, Ada.Text_IO;


with Simulator_operator;
use Simulator_operator;

package body GA_operator is

   type Controller_inf is
      record
         Kp, Ki, Kd : Real;
         score : Real;
         evaluated: boolean;
      end record;

   seed : Ada.Numerics.Float_Random.Generator;

   Max_population : constant integer := 10000;

   Controller: array (0..1,1..Max_population) of Controller_inf;

   procedure Init_population is
      temp_controller : Controller_inf;
      exist : boolean;
   begin
      Ada.Numerics.Float_Random.Reset(seed);

      for x in 1..Max_population loop
         loop
            temp_controller.Kp := Real(Random(seed))*0.01;
            temp_controller.Ki := Real(Random(seed))*0.00001;
            temp_controller.Kd := 0.00001;
            temp_controller.score := Real'last/2.0;
            temp_controller.evaluated := false;

            exist := false;
            if x > 1 then
               for y in 1..(x-1) loop
                  if temp_controller = Controller(0,y) then
                     exist := true;
                  end if;
               end loop;
            end if;
            exit when not exist;
         end loop;

         Controller(0,x) := temp_controller;
      end loop;

   end Init_population;

   procedure combination is
      y,z : integer;
      param : Real;
      temp_controller : Controller_inf;
      exist : boolean;
   begin
      -- Combine between elite elements
      for x in (Max_population/10 + 1)..2*Max_population/10 loop

         loop
            -- Select elements
            loop
               y := integer(Float'Truncation(Float(Random(seed))*Float(Max_population/10)));
               exit when y > 0;
            end loop;

            loop
               z := integer(Float'Truncation(Float(Random(seed))*Float(Max_population/10)));
               exit when (z /= y) and (z > 0);
            end loop;

            -- Select parameter of z element to combine with y parameters
            param := Real(Random(seed));
            temp_controller := Controller(1,x);

            if param < 0.3333 then
               temp_controller.Kp := Controller(0,z).Kp;
               temp_controller.Ki := Controller(0,y).Ki;
               temp_controller.Kd := Controller(0,y).Kd;
            elsif param < 0.6666 then
               temp_controller.Kp := Controller(0,y).Kp;
               temp_controller.Ki := Controller(0,z).Ki;
               temp_controller.Kd := Controller(0,y).Kd;
            else
               temp_controller.Kp := Controller(0,y).Kp;
               temp_controller.Ki := Controller(0,y).Ki;
               temp_controller.Kd := Controller(0,z).Kd;
            end if;

            temp_controller.score := Real'last/2.0;
            temp_controller.evaluated := false;

            -- Verify if there exists a twin
            exist := false;
            for q in 1..(x-1) loop
               if (Controller(1,q).Kp = temp_controller.Kp) and
                 (Controller(1,q).Ki = temp_controller.Ki) and
                 (Controller(1,q).Kd = temp_controller.Kd) then

                  exist := true;
               end if;
               exit when exist;
            end loop;

            exit when not exist;
         end loop;

         Controller(1,x) := temp_controller;
      end loop;

      -- Combine elite elements with general elements
      for x in (2*Max_population/10 + 1)..5*Max_population/10 loop
         loop
            -- Select elements
            loop
               y := integer(Float'Truncation(Random(seed)*Float(Max_population/10)));
               exit when y /= 0;
            end loop;

            loop
               z := integer(Float'Truncation(Random(seed)*Float(Max_population)));
               exit when (z /= y) and (z > 0);
            end loop;

            -- Select parameter of z element to combine with y parameters
            param := Real(Random(seed));
            temp_controller := Controller(1,x);

            if param < 0.3333 then
               temp_controller.Kp := Controller(0,z).Kp;
               temp_controller.Ki := Controller(0,y).Ki;
               temp_controller.Kd := Controller(0,y).Kd;
            elsif param < 0.6666 then
               temp_controller.Kp := Controller(0,y).Kp;
               temp_controller.Ki := Controller(0,z).Ki;
               temp_controller.Kd := Controller(0,y).Kd;
            else
               temp_controller.Kp := Controller(0,y).Kp;
               temp_controller.Ki := Controller(0,y).Ki;
               temp_controller.Kd := Controller(0,z).Kd;
            end if;

            temp_controller.score := Real'last/2.0;
            temp_controller.evaluated := false;

            -- Verify if there exists a twin
            exist := false;
            for q in 1..(x-1) loop
               if (Controller(1,q).Kp = temp_controller.Kp) and
                 (Controller(1,q).Ki = temp_controller.Ki) and
                 (Controller(1,q).Kd = temp_controller.Kd) then

                  exist := true;
               end if;
               exit when exist;
            end loop;

            exit when not exist;
         end loop;

         Controller(1,x) := temp_controller;
      end loop;

      -- Combine general elements
      for x in (5*Max_population/10 + 1)..7*Max_population/10 loop
         loop
            -- Select elements
            loop
               y := integer(Float'Truncation(Random(seed)*Float(Max_population)));
               exit when y /= 0;
            end loop;

            loop
               z := integer(Float'Truncation(Random(seed)*Float(Max_population)));
               exit when (z /= y) and (z > 0);
            end loop;

            -- Select parameter of z element to combine with y parameters
            param := Real(Random(seed));
            temp_controller := Controller(1,x);

            if param < 0.3333 then
               temp_controller.Kp := Controller(0,z).Kp;
               temp_controller.Ki := Controller(0,y).Ki;
               temp_controller.Kd := Controller(0,y).Kd;
            elsif param < 0.6666 then
               temp_controller.Kp := Controller(0,y).Kp;
               temp_controller.Ki := Controller(0,z).Ki;
               temp_controller.Kd := Controller(0,y).Kd;
            else
               temp_controller.Kp := Controller(0,y).Kp;
               temp_controller.Ki := Controller(0,y).Ki;
               temp_controller.Kd := Controller(0,z).Kd;
            end if;

            temp_controller.score := Real'last/2.0;
            temp_controller.evaluated := false;

            -- Verify if there exists a twin
            exist := false;
            for q in 1..(x-1) loop
               if (Controller(1,q).Kp = temp_controller.Kp) and
                 (Controller(1,q).Ki = temp_controller.Ki) and
                 (Controller(1,q).Kd = temp_controller.Kd) then

                  exist := true;
               end if;
               exit when exist;
            end loop;

            exit when not exist;
         end loop;

         Controller(1,x) := temp_controller;
      end loop;

   end Combination;

   procedure Mutation is
      y : integer;
      param, severity : Real;
      sign : Real;

      temp_controller : Controller_inf;
      exist : boolean;
   begin
      for x in (7*Max_population/10 + 1)..Max_population loop
         loop
            -- Select an element
            loop
               y := integer(Float'Truncation(Random(seed)*Float(Max_population)));
               exit when y /= 0;
            end loop;

            -- Select a parameter to mutate
            param := Real(Random(seed));
            -- Select severity of mutation
            severity := Real(Random(seed));
            -- Select de sign of the mutation
            sign := Real(Random(seed));

            temp_controller := Controller(0,y);

            if sign >= 0.5 then
               if param < 0.3333 then
                  if severity < 0.3333 then
                     temp_controller.Kp := temp_controller.Kp + Real(Random(seed))*0.001;
                  elsif severity < 0.6666 then
                     temp_controller.Kp := temp_controller.Kp + Real(Random(Seed))*0.1;
                  else
                     temp_controller.Kp := temp_controller.Kp + Real(Random(Seed))*10.0;
                  end if;
               elsif param < 0.6666 then
                  if severity < 0.3333 then
                     temp_controller.Ki := temp_controller.Ki + Real(Random(seed))*0.001;
                  elsif severity < 0.6666 then
                     temp_controller.Ki := temp_controller.Ki + Real(Random(Seed))*0.1;
                  else
                     temp_controller.Ki := temp_controller.Ki + Real(Random(Seed))*10.0;
                  end if;
               else
                  if severity < 0.3333 then
                     temp_controller.Kd := temp_controller.Kd + Real(Random(seed))*0.00000001;
                  elsif severity < 0.6666 then
                     temp_controller.Kd := temp_controller.Kd + Real(Random(Seed))*0.0000001;
                  else
                     temp_controller.Kd := temp_controller.Kd + Real(Random(Seed))*0.0001;
                  end if;
               end if;
            else
               if param < 0.3333 then
                  if severity < 0.3333 then
                     temp_controller.Kp := temp_controller.Kp - Real(Random(seed))*0.001;
                  elsif severity < 0.6666 then
                     temp_controller.Kp := temp_controller.Kp - Real(Random(Seed))*0.1;
                  else
                     temp_controller.Kp := temp_controller.Kp - Real(Random(Seed))*10.0;
                  end if;
               elsif param < 0.6666 then
                  if severity < 0.3333 then
                     temp_controller.Ki := temp_controller.Ki - Real(Random(seed))*0.001;
                  elsif severity < 0.6666 then
                     temp_controller.Ki := temp_controller.Ki - Real(Random(Seed))*0.1;
                  else
                     temp_controller.Ki := temp_controller.Ki - Real(Random(Seed))*10.0;
                  end if;
               else
                  if severity < 0.3333 then
                     temp_controller.Kd := temp_controller.Kd - Real(Random(seed))*0.00000001;
                  elsif severity < 0.6666 then
                     temp_controller.Kd := temp_controller.Kd - Real(Random(Seed))*0.0000001;
                  else
                     temp_controller.Kd := temp_controller.Kd - Real(Random(Seed))*0.0001;
                  end if;
               end if;
            end if;

            if temp_controller.Kp < 0.0 then
               temp_controller.Kp := temp_controller.Kp * (-1.0);
            end if;

            if temp_controller.Ki < 0.0 then
               temp_controller.Ki := temp_controller.Ki * (-1.0);
            end if;

            if temp_controller.Kd < 0.0 then
               temp_controller.Kd := temp_controller.Kd * (-1.0);
            end if;

            temp_controller.score := Real'last/2.0;
            temp_controller.evaluated := false;

            -- Verify if there exists a twin
            exist := false;
            for q in 1..(x-1) loop
               if (Controller(1,q).Kp = temp_controller.Kp) and
                 (Controller(1,q).Ki = temp_controller.Ki) and
                 (Controller(1,q).Kd = temp_controller.Kd) then

                  exist := true;
               end if;
               exit when exist;
            end loop;

            exit when not exist;
         end loop;

         Controller(1,x) := temp_controller;
      end loop;

   end Mutation;

   procedure Run_GA is

      Generation : integer := 0;
      Score : Real;

      Changes : boolean;
      temp_controller : Controller_inf;

      package Real_io is new Ada.Text_IO.Fixed_IO(Real);
      package integer_io is new Ada.Text_IO.Integer_IO(integer);
      use Real_IO, Integer_IO;

   begin
      Init_population;

      loop
         -- Evaluate all the controllers
         Generation := Generation +1;
         put("Generation: ");
         put(Generation);
         new_line;

         for x in 1..Max_population loop
            if Controller(0,x).evaluated = false then
               Carry_out_a_simulation(Controller(0,x).Kp, Controller(0,x).Ki,
                                      Controller(0,x).Kd, Score);
               Controller(0,x).score := Score;
               Controller(0,x).evaluated := true;
            end if;

            -- Trace
            put(Generation);
            put(Controller(0,x).Kp);
            put(Controller(0,x).Ki);
            put(Controller(0,x).Kd);
            put(Controller(0,x).score);
            new_line;
         end loop;

         -- Sort population (bubble method)
         Changes := false;

         loop
            for x in 1..(Max_population - 1) loop
               if Controller(0,x).Score > Controller(0,x+1).Score then
                  Changes := true;
                  temp_controller := Controller(0,x);
                  Controller(0,x) := Controller(0,x+1);
                  Controller(0,x+1) := temp_controller;
               end if;
            end loop;

            exit when not Changes;
            Changes := false;
         end loop;

         -- Finish AG when ...
         exit when (Controller(0,1).Score < 10.0) or (Generation > 5000) ;

         -- Prepare new generation in Controller(1,x)

         -- Select with elitism 10% of new population
         for x in 1..Max_population/10 loop
            Controller(1,x) := Controller(0,x);
            -- Mantain evaluated
         end loop;


         Combination;

         Mutation;

         -- Copy Controller(1,x) in Controller(0,x)
         for x in 1..Max_population loop
            Controller(0,x) := Controller(1,x);
         end loop;

      end loop;

      -- Show the best controller
      new_line;
      put("The best controller is:");
      new_line;
      new_line;
      put(Controller(0,1).Kp);
      put(Controller(0,1).Ki);
      put(Controller(0,1).Kd);
      put(Controller(0,1).score);
      new_line;
      new_line;
      Put("Generation: ");
      Put(Generation);

   end Run_GA;

end GA_operator;
