for ((R=1; R<=33; ++R)) do
  ./test_alldiff_reLOC 32 34 $R bijection > alldiff_32_34_$R-bij.cnf
done