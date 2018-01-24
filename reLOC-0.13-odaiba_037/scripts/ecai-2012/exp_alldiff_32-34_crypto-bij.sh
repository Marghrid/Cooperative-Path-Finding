for ((R=1; R<=33; ++R)) do
  echo $R
 ../../sat/cryptominisat alldiff_32_34_$R-bij.cnf
done