for ((A=1; A<=1022; ++A)) do
  ./resolver_reLOC --solution-file=../../../multirobot/experiments/coin-2012/random/32x32/bibox_grid_32x32#$A\_redundant.txt --window-size=16 --output-file=whca_grid_32x32#$A\_solution.txt > whca_grid_32x32#$A.txt
done
