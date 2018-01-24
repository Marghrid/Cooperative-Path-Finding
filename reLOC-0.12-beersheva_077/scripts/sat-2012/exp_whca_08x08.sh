for ((A=1; A<=62; ++A)) do
  ./resolver_reLOC --solution-file=../../../multirobot/experiments/coin-2012/random/08x08/bibox_grid_8x8#$A\_redundant.txt --window-size=16 --output-file=whca_grid_8x8#$A\_solution.txt > whca_grid_8x8#$A.txt
done
