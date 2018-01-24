for ((R=16; R<=33; ++R)) do
  echo $R
 ../../sat/clasp alldiff_32_34_$R-bij.cnf
done