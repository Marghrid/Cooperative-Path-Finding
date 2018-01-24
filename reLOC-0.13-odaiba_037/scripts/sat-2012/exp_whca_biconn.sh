#for ((A=1; A<=60; ++A)) do
#  ./resolver_reLOC --solution-file=../../../multirobot/experiments/coin-2012/random/biconn_04/biconn_7-112-4#$A\_redundant.txt --window-size=16 --output-file=whca_biconn_04#$A\_solution.txt > whca_biconn_04#$A.txt
#done

#for ((A=1; A<=60; ++A)) do
#  ./resolver_reLOC --solution-file=../../../multirobot/experiments/coin-2012/random/biconn_08/biconn_9-68-8#$A\_redundant.txt --window-size=16 --output-file=whca_biconn_08#$A\_solution.txt > whca_biconn_08#$A.txt
#done

#for ((A=1; A<=60; ++A)) do
#  ./resolver_reLOC --solution-file=../../../multirobot/experiments/coin-2012/random/biconn_16/biconn_17-33-16#$A\_redundant.txt --window-size=16 --output-file=whca_biconn_16#$A\_solution.txt > whca_biconn_16#$A.txt
#done

for ((A=1; A<=60; ++A)) do
  ./resolver_reLOC --solution-file=../../../multirobot/experiments/coin-2012/random/biconn_32/biconn_33-20-32#$A\_redundant.txt --window-size=16 --output-file=whca_biconn_32#$A\_solution.txt > whca_biconn_32#$A.txt
done

