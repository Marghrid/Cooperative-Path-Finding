./resolver_reLOC --window-size=16 --output-file=../../whca/16x16/random/whca_grid_16x16#1.txt --solution-file=../../bibox/16x16/random/bibox_grid_16x16#1_redundant.txt > ../../experiments/bibox_2_whca_grid_16x16#1.txt
./resolver_reLOC --window-size=16 --output-file=../../whca/16x16/random/whca_grid_16x16#4.txt --solution-file=../../bibox/16x16/random/bibox_grid_16x16#4_redundant.txt > ../../experiments/bibox_2_whca_grid_16x16#4.txt
./resolver_reLOC --window-size=16 --output-file=../../whca/16x16/random/whca_grid_16x16#8.txt --solution-file=../../bibox/16x16/random/bibox_grid_16x16#8_redundant.txt > ../../experiments/bibox_2_whca_grid_16x16#8.txt
./resolver_reLOC --window-size=16 --output-file=../../whca/16x16/random/whca_grid_16x16#12.txt --solution-file=../../bibox/16x16/random/bibox_grid_16x16#12_redundant.txt > ../../experiments/bibox_2_whca_grid_16x16#12.txt
./resolver_reLOC --window-size=16 --output-file=../../whca/16x16/random/whca_grid_16x16#16.txt --solution-file=../../bibox/16x16/random/bibox_grid_16x16#16_redundant.txt > ../../experiments/bibox_2_whca_grid_16x16#16.txt
./resolver_reLOC --window-size=16 --output-file=../../whca/16x16/random/whca_grid_16x16#20.txt --solution-file=../../bibox/16x16/random/bibox_grid_16x16#20_redundant.txt > ../../experiments/bibox_2_whca_grid_16x16#20.txt
./resolver_reLOC --window-size=16 --output-file=../../whca/16x16/random/whca_grid_16x16#24.txt --solution-file=../../bibox/16x16/random/bibox_grid_16x16#24_redundant.txt > ../../experiments/bibox_2_whca_grid_16x16#24.txt
./resolver_reLOC --window-size=16 --output-file=../../whca/16x16/random/whca_grid_16x16#28.txt --solution-file=../../bibox/16x16/random/bibox_grid_16x16#28_redundant.txt > ../../experiments/bibox_2_whca_grid_16x16#28.txt
./resolver_reLOC --window-size=16 --output-file=../../whca/16x16/random/whca_grid_16x16#32.txt --solution-file=../../bibox/16x16/random/bibox_grid_16x16#32_redundant.txt > ../../experiments/bibox_2_whca_grid_16x16#32.txt
./resolver_reLOC --window-size=16 --output-file=../../whca/16x16/random/whca_grid_16x16#36.txt --solution-file=../../bibox/16x16/random/bibox_grid_16x16#36_redundant.txt > ../../experiments/bibox_2_whca_grid_16x16#36.txt
./resolver_reLOC --window-size=16 --output-file=../../whca/16x16/random/whca_grid_16x16#40.txt --solution-file=../../bibox/16x16/random/bibox_grid_16x16#40_redundant.txt > ../../experiments/bibox_2_whca_grid_16x16#40.txt
./resolver_reLOC --window-size=16 --output-file=../../whca/16x16/random/whca_grid_16x16#44.txt --solution-file=../../bibox/16x16/random/bibox_grid_16x16#44_redundant.txt > ../../experiments/bibox_2_whca_grid_16x16#44.txt
./resolver_reLOC --window-size=16 --output-file=../../whca/16x16/random/whca_grid_16x16#48.txt --solution-file=../../bibox/16x16/random/bibox_grid_16x16#48_redundant.txt > ../../experiments/bibox_2_whca_grid_16x16#48.txt
./resolver_reLOC --window-size=16 --output-file=../../whca/16x16/random/whca_grid_16x16#52.txt --solution-file=../../bibox/16x16/random/bibox_grid_16x16#52_redundant.txt > ../../experiments/bibox_2_whca_grid_16x16#52.txt
./resolver_reLOC --window-size=16 --output-file=../../whca/16x16/random/whca_grid_16x16#56.txt --solution-file=../../bibox/16x16/random/bibox_grid_16x16#56_redundant.txt > ../../experiments/bibox_2_whca_grid_16x16#56.txt
./resolver_reLOC --window-size=16 --output-file=../../whca/16x16/random/whca_grid_16x16#60.txt --solution-file=../../bibox/16x16/random/bibox_grid_16x16#60_redundant.txt > ../../experiments/bibox_2_whca_grid_16x16#60.txt
./resolver_reLOC --window-size=16 --output-file=../../whca/16x16/random/whca_grid_16x16#64.txt --solution-file=../../bibox/16x16/random/bibox_grid_16x16#64_redundant.txt > ../../experiments/bibox_2_whca_grid_16x16#64.txt
./resolver_reLOC --window-size=16 --output-file=../../whca/16x16/random/whca_grid_16x16#68.txt --solution-file=../../bibox/16x16/random/bibox_grid_16x16#68_redundant.txt > ../../experiments/bibox_2_whca_grid_16x16#68.txt


./optimizer_reLOC --minisat-timeout=240 --makespan-bound=10 --number-of-threads=8 --solution-file=../../whca/16x16/random/whca_grid_16x16#1.txt > ../../experiments/whca_grid_16x16#1_optimized.txt
./optimizer_reLOC --minisat-timeout=240 --makespan-bound=10 --number-of-threads=8 --solution-file=../../whca/16x16/random/whca_grid_16x16#4.txt > ../../experiments/whca_grid_16x16#4_optimized.txt
./optimizer_reLOC --minisat-timeout=240 --makespan-bound=10 --number-of-threads=8 --solution-file=../../whca/16x16/random/whca_grid_16x16#8.txt > ../../experiments/whca_grid_16x16#8_optimized.txt
./optimizer_reLOC --minisat-timeout=240 --makespan-bound=10 --number-of-threads=8 --solution-file=../../whca/16x16/random/whca_grid_16x16#12.txt > ../../experiments/whca_grid_16x16#12_optimized.txt
./optimizer_reLOC --minisat-timeout=240 --makespan-bound=10 --number-of-threads=8 --solution-file=../../whca/16x16/random/whca_grid_16x16#16.txt > ../../experiments/whca_grid_16x16#16_optimized.txt
./optimizer_reLOC --minisat-timeout=240 --makespan-bound=10 --number-of-threads=8 --solution-file=../../whca/16x16/random/whca_grid_16x16#20.txt > ../../experiments/whca_grid_16x16#20_optimized.txt
./optimizer_reLOC --minisat-timeout=240 --makespan-bound=10 --number-of-threads=8 --solution-file=../../whca/16x16/random/whca_grid_16x16#24.txt > ../../experiments/whca_grid_16x16#24_optimized.txt
./optimizer_reLOC --minisat-timeout=240 --makespan-bound=10 --number-of-threads=8 --solution-file=../../whca/16x16/random/whca_grid_16x16#28.txt > ../../experiments/whca_grid_16x16#28_optimized.txt
./optimizer_reLOC --minisat-timeout=240 --makespan-bound=10 --number-of-threads=8 --solution-file=../../whca/16x16/random/whca_grid_16x16#32.txt > ../../experiments/whca_grid_16x16#32_optimized.txt
./optimizer_reLOC --minisat-timeout=240 --makespan-bound=10 --number-of-threads=8 --solution-file=../../whca/16x16/random/whca_grid_16x16#36.txt > ../../experiments/whca_grid_16x16#36_optimized.txt
./optimizer_reLOC --minisat-timeout=240 --makespan-bound=10 --number-of-threads=8 --solution-file=../../whca/16x16/random/whca_grid_16x16#40.txt > ../../experiments/whca_grid_16x16#40_optimized.txt
./optimizer_reLOC --minisat-timeout=240 --makespan-bound=10 --number-of-threads=8 --solution-file=../../whca/16x16/random/whca_grid_16x16#44.txt > ../../experiments/whca_grid_16x16#44_optimized.txt
./optimizer_reLOC --minisat-timeout=240 --makespan-bound=10 --number-of-threads=8 --solution-file=../../whca/16x16/random/whca_grid_16x16#48.txt > ../../experiments/whca_grid_16x16#48_optimized.txt
./optimizer_reLOC --minisat-timeout=240 --makespan-bound=10 --number-of-threads=8 --solution-file=../../whca/16x16/random/whca_grid_16x16#52.txt > ../../experiments/whca_grid_16x16#52_optimized.txt
./optimizer_reLOC --minisat-timeout=240 --makespan-bound=10 --number-of-threads=8 --solution-file=../../whca/16x16/random/whca_grid_16x16#56.txt > ../../experiments/whca_grid_16x16#56_optimized.txt
./optimizer_reLOC --minisat-timeout=240 --makespan-bound=10 --number-of-threads=8 --solution-file=../../whca/16x16/random/whca_grid_16x16#60.txt > ../../experiments/whca_grid_16x16#60_optimized.txt
./optimizer_reLOC --minisat-timeout=240 --makespan-bound=10 --number-of-threads=8 --solution-file=../../whca/16x16/random/whca_grid_16x16#64.txt > ../../experiments/whca_grid_16x16#64_optimized.txt
./optimizer_reLOC --minisat-timeout=240 --makespan-bound=10 --number-of-threads=8 --solution-file=../../whca/16x16/random/whca_grid_16x16#68.txt > ../../experiments/whca_grid_16x16#68_optimized.txt
