for ((R=19; R<=33; ++R)) do
  echo $R
 ../../sat/cryptominisat alldiff_32_34_$R-std.cnf
done