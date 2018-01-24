for ((A=1; A<=254; ++A)) do
  ./resolver_reLOC --solution-file=../../../multirobot/experiments/coin-2012/random/16x16/bibox_grid_16x16#$A\_redundant.txt --window-size=16 --output-file=whca_grid_16x16#$A\_solution.txt > whca_grid_16x16#$A.txt
done
