for ((R=1; R<=33; ++R)) do
  ./test_alldiff_reLOC 32 34 $R bijection > alldiff_32_34_$R-bij.cnf
done

#for ((R=1; R<=33; ++R)) do
# ../../sat/minisat_static -cpu-lim=3600 alldiff_32_34_$R-bij.cnf
#done

#for ((R=1; R<=33; ++R)) do
# ../../sat/glucose_static -cpu-lim=3600 alldiff_32_34_$R-bij.cnf
#done

for ((R=1; R<=33; ++R)) do
  echo $R
 ../../sat/clasp alldiff_32_34_$R-bij.cnf
done

for ((R=1; R<=33; ++R)) do
  echo $R
 ../../sat/glueminisat_static alldiff_32_34_$R-bij.cnf
done

for ((R=16; R<=33; ++R)) do
  echo $R
 ../../sat/precosat alldiff_32_34_$R-bij.cnf
done