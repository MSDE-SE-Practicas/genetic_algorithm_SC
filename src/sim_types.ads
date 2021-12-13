package sim_types is

   type Real is delta 0.000000001
     range -((2 ** 63 - 1) * 0.000000001) ..
           +((2 ** 63 - 1) * 0.000000001);
   for Real'Small use 0.000000001;

end sim_types;
