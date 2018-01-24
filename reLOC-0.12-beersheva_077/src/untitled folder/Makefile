all: debug

debug:	search.o_dbg complete.o_dbg types.o_dbg compress_test.o_dbg gridgen_main.o_dbg optimizer_main.o_dbg solver_main.o_dbg cnf_test.o_dbg hierarch.o_dbg compress.o_dbg resolver_main.o_dbg graph.o_dbg cnf.o_dbg autooptim_main.o_dbg netgen_main.o_dbg complete_test.o_dbg hierarch_test.o_dbg hypergen_main.o_dbg alldiff_test.o_dbg multirobot_test.o_dbg reloc.o_dbg multirobot.o_dbg simplify_main.o_dbg statistics.o_dbg reloc_test.o_dbg 
	g++ -lpthread -otest_basic_reLOC types.o_dbg reloc.o_dbg hierarch.o_dbg multirobot.o_dbg search.o_dbg cnf.o_dbg compress.o_dbg statistics.o_dbg graph.o_dbg complete.o_dbg reloc_test.o_dbg
	g++ -lpthread -otest_multirobot_reLOC types.o_dbg reloc.o_dbg hierarch.o_dbg multirobot.o_dbg search.o_dbg cnf.o_dbg compress.o_dbg graph.o_dbg complete.o_dbg statistics.o_dbg multirobot_test.o_dbg
	g++ -lpthread -otest_hierarch_reLOC types.o_dbg reloc.o_dbg hierarch.o_dbg multirobot.o_dbg search.o_dbg cnf.o_dbg compress.o_dbg statistics.o_dbg graph.o_dbg complete.o_dbg hierarch_test.o_dbg
	g++ -lpthread -otest_cnf_reLOC types.o_dbg reloc.o_dbg hierarch.o_dbg multirobot.o_dbg search.o_dbg cnf.o_dbg compress.o_dbg statistics.o_dbg graph.o_dbg complete.o_dbg cnf_test.o_dbg
	g++ -lpthread -otest_alldiff_reLOC types.o_dbg reloc.o_dbg hierarch.o_dbg multirobot.o_dbg search.o_dbg cnf.o_dbg compress.o_dbg statistics.o_dbg graph.o_dbg complete.o_dbg alldiff_test.o_dbg
	g++ -lpthread -otest_compress_reLOC types.o_dbg reloc.o_dbg hierarch.o_dbg multirobot.o_dbg search.o_dbg cnf.o_dbg compress.o_dbg statistics.o_dbg graph.o_dbg complete.o_dbg compress_test.o_dbg
	g++ -lpthread -otest_complete_reLOC types.o_dbg reloc.o_dbg hierarch.o_dbg multirobot.o_dbg search.o_dbg cnf.o_dbg compress.o_dbg statistics.o_dbg graph.o_dbg complete.o_dbg complete_test.o_dbg
	g++ -lpthread -ooptimizer_reLOC types.o_dbg reloc.o_dbg hierarch.o_dbg multirobot.o_dbg search.o_dbg cnf.o_dbg compress.o_dbg statistics.o_dbg graph.o_dbg complete.o_dbg optimizer_main.o_dbg
	g++ -lpthread -oautooptim_reLOC types.o_dbg reloc.o_dbg hierarch.o_dbg multirobot.o_dbg search.o_dbg cnf.o_dbg compress.o_dbg statistics.o_dbg graph.o_dbg complete.o_dbg autooptim_main.o_dbg
	g++ -lpthread -oresolver_reLOC types.o_dbg reloc.o_dbg hierarch.o_dbg multirobot.o_dbg search.o_dbg cnf.o_dbg compress.o_dbg statistics.o_dbg graph.o_dbg complete.o_dbg resolver_main.o_dbg
	g++ -lpthread -osolver_reLOC types.o_dbg reloc.o_dbg hierarch.o_dbg multirobot.o_dbg search.o_dbg cnf.o_dbg compress.o_dbg statistics.o_dbg graph.o_dbg complete.o_dbg solver_main.o_dbg
	g++ -lpthread -ogridgen_reLOC types.o_dbg reloc.o_dbg hierarch.o_dbg multirobot.o_dbg search.o_dbg cnf.o_dbg compress.o_dbg statistics.o_dbg graph.o_dbg complete.o_dbg gridgen_main.o_dbg
	g++ -lpthread -onetgen_reLOC types.o_dbg reloc.o_dbg hierarch.o_dbg multirobot.o_dbg search.o_dbg cnf.o_dbg compress.o_dbg statistics.o_dbg graph.o_dbg complete.o_dbg netgen_main.o_dbg
	g++ -lpthread -ohypergen_reLOC types.o_dbg reloc.o_dbg hierarch.o_dbg multirobot.o_dbg search.o_dbg cnf.o_dbg compress.o_dbg statistics.o_dbg graph.o_dbg complete.o_dbg hypergen_main.o_dbg
	g++ -lpthread -osimplify_reLOC types.o_dbg reloc.o_dbg hierarch.o_dbg multirobot.o_dbg search.o_dbg cnf.o_dbg compress.o_dbg statistics.o_dbg graph.o_dbg complete.o_dbg simplify_main.o_dbg

optimized:	hierarch_test.o_opt optimizer_main.o_opt solver_main.o_opt search.o_opt cnf.o_opt reloc_test.o_opt multirobot.o_opt compress.o_opt hierarch.o_opt complete.o_opt netgen_main.o_opt hypergen_main.o_opt alldiff_test.o_opt types.o_opt multirobot_test.o_opt reloc.o_opt statistics.o_opt gridgen_main.o_opt graph.o_opt autooptim_main.o_opt compress_test.o_opt resolver_main.o_opt cnf_test.o_opt simplify_main.o_opt complete_test.o_opt 
	g++ -lpthread -otest_basic_reLOC types.o_opt reloc.o_opt hierarch.o_opt multirobot.o_opt search.o_opt cnf.o_opt compress.o_opt statistics.o_opt graph.o_opt complete.o_opt reloc_test.o_opt
	g++ -lpthread -otest_multirobot_reLOC types.o_opt reloc.o_opt hierarch.o_opt multirobot.o_opt search.o_opt cnf.o_opt compress.o_opt graph.o_opt complete.o_opt statistics.o_opt multirobot_test.o_opt
	g++ -lpthread -otest_hierarch_reLOC types.o_opt reloc.o_opt hierarch.o_opt multirobot.o_opt search.o_opt cnf.o_opt compress.o_opt statistics.o_opt graph.o_opt complete.o_opt hierarch_test.o_opt
	g++ -lpthread -otest_cnf_reLOC types.o_opt reloc.o_opt hierarch.o_opt multirobot.o_opt search.o_opt cnf.o_opt compress.o_opt statistics.o_opt graph.o_opt complete.o_opt cnf_test.o_opt
	g++ -lpthread -otest_alldiff_reLOC types.o_opt reloc.o_opt hierarch.o_opt multirobot.o_opt search.o_opt cnf.o_opt compress.o_opt statistics.o_opt graph.o_opt complete.o_opt alldiff_test.o_opt
	g++ -lpthread -otest_compress_reLOC types.o_opt reloc.o_opt hierarch.o_opt multirobot.o_opt search.o_opt cnf.o_opt compress.o_opt statistics.o_opt graph.o_opt complete.o_opt compress_test.o_opt
	g++ -lpthread -otest_complete_reLOC types.o_opt reloc.o_opt hierarch.o_opt multirobot.o_opt search.o_opt cnf.o_opt compress.o_opt statistics.o_opt graph.o_opt complete.o_opt complete_test.o_opt
	g++ -lpthread -ooptimizer_reLOC types.o_opt reloc.o_opt hierarch.o_opt multirobot.o_opt search.o_opt cnf.o_opt compress.o_opt statistics.o_opt graph.o_opt complete.o_opt optimizer_main.o_opt
	g++ -lpthread -oautooptim_reLOC types.o_opt reloc.o_opt hierarch.o_opt multirobot.o_opt search.o_opt cnf.o_opt compress.o_opt statistics.o_opt graph.o_opt complete.o_opt autooptim_main.o_opt
	g++ -lpthread -oresolver_reLOC types.o_opt reloc.o_opt hierarch.o_opt multirobot.o_opt search.o_opt cnf.o_opt compress.o_opt statistics.o_opt graph.o_opt complete.o_opt resolver_main.o_opt
	g++ -lpthread -osolver_reLOC types.o_opt reloc.o_opt hierarch.o_opt multirobot.o_opt search.o_opt cnf.o_opt compress.o_opt statistics.o_opt graph.o_opt complete.o_opt solver_main.o_opt
	g++ -lpthread -ogridgen_reLOC types.o_opt reloc.o_opt hierarch.o_opt multirobot.o_opt search.o_opt cnf.o_opt compress.o_opt statistics.o_opt graph.o_opt complete.o_opt gridgen_main.o_opt
	g++ -lpthread -onetgen_reLOC types.o_opt reloc.o_opt hierarch.o_opt multirobot.o_opt search.o_opt cnf.o_opt compress.o_opt statistics.o_opt graph.o_opt complete.o_opt netgen_main.o_opt
	g++ -lpthread -ohypergen_reLOC types.o_opt reloc.o_opt hierarch.o_opt multirobot.o_opt search.o_opt cnf.o_opt compress.o_opt statistics.o_opt graph.o_opt complete.o_opt hypergen_main.o_opt
	g++ -lpthread -osimplify_reLOC types.o_opt reloc.o_opt hierarch.o_opt multirobot.o_opt search.o_opt cnf.o_opt compress.o_opt statistics.o_opt graph.o_opt complete.o_opt simplify_main.o_opt

types.o_dbg:	version.h defs.h config.h compile.h types.h result.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h reloc_test.h types.cpp
	g++ -Wall -Wextra -pedantic -Wno-long-long -Wno-sign-compare -g -c -otypes.o_dbg types.cpp

reloc.o_dbg:	version.h defs.h config.h compile.h types.h result.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h reloc_test.h reloc.cpp
	g++ -Wall -Wextra -pedantic -Wno-long-long -Wno-sign-compare -g -c -oreloc.o_dbg reloc.cpp

hierarch.o_dbg:	version.h defs.h config.h compile.h types.h result.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h reloc_test.h hierarch.cpp
	g++ -Wall -Wextra -pedantic -Wno-long-long -Wno-sign-compare -g -c -ohierarch.o_dbg hierarch.cpp

multirobot.o_dbg:	version.h defs.h config.h compile.h types.h result.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h reloc_test.h multirobot.cpp
	g++ -Wall -Wextra -pedantic -Wno-long-long -Wno-sign-compare -g -c -omultirobot.o_dbg multirobot.cpp

search.o_dbg:	version.h defs.h config.h compile.h types.h result.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h reloc_test.h search.cpp
	g++ -Wall -Wextra -pedantic -Wno-long-long -Wno-sign-compare -g -c -osearch.o_dbg search.cpp

cnf.o_dbg:	version.h defs.h config.h compile.h types.h result.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h reloc_test.h cnf.cpp
	g++ -Wall -Wextra -pedantic -Wno-long-long -Wno-sign-compare -g -c -ocnf.o_dbg cnf.cpp

compress.o_dbg:	version.h defs.h config.h compile.h types.h result.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h reloc_test.h compress.cpp
	g++ -Wall -Wextra -pedantic -Wno-long-long -Wno-sign-compare -g -c -ocompress.o_dbg compress.cpp

statistics.o_dbg:	version.h defs.h config.h compile.h types.h result.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h reloc_test.h statistics.cpp
	g++ -Wall -Wextra -pedantic -Wno-long-long -Wno-sign-compare -g -c -ostatistics.o_dbg statistics.cpp

graph.o_dbg:	version.h defs.h config.h compile.h types.h result.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h reloc_test.h graph.cpp
	g++ -Wall -Wextra -pedantic -Wno-long-long -Wno-sign-compare -g -c -ograph.o_dbg graph.cpp

complete.o_dbg:	version.h defs.h config.h compile.h types.h result.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h reloc_test.h complete.cpp
	g++ -Wall -Wextra -pedantic -Wno-long-long -Wno-sign-compare -g -c -ocomplete.o_dbg complete.cpp

reloc_test.o_dbg:	version.h defs.h config.h compile.h types.h result.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h reloc_test.h reloc_test.cpp
	g++ -Wall -Wextra -pedantic -Wno-long-long -Wno-sign-compare -g -c -oreloc_test.o_dbg reloc_test.cpp

multirobot_test.o_dbg:	version.h defs.h config.h compile.h types.h result.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h graph.h complete.h statistics.h multirobot_test.h multirobot_test.cpp
	g++ -Wall -Wextra -pedantic -Wno-long-long -Wno-sign-compare -g -c -omultirobot_test.o_dbg multirobot_test.cpp

hierarch_test.o_dbg:	version.h defs.h config.h compile.h types.h result.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h hierarch_test.h hierarch_test.cpp
	g++ -Wall -Wextra -pedantic -Wno-long-long -Wno-sign-compare -g -c -ohierarch_test.o_dbg hierarch_test.cpp

cnf_test.o_dbg:	version.h defs.h config.h compile.h types.h result.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h cnf_test.h cnf_test.cpp
	g++ -Wall -Wextra -pedantic -Wno-long-long -Wno-sign-compare -g -c -ocnf_test.o_dbg cnf_test.cpp

alldiff_test.o_dbg:	version.h defs.h config.h compile.h types.h result.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h alldiff_test.h alldiff_test.cpp
	g++ -Wall -Wextra -pedantic -Wno-long-long -Wno-sign-compare -g -c -oalldiff_test.o_dbg alldiff_test.cpp

compress_test.o_dbg:	version.h defs.h config.h compile.h types.h result.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h compress_test.h compress_test.cpp
	g++ -Wall -Wextra -pedantic -Wno-long-long -Wno-sign-compare -g -c -ocompress_test.o_dbg compress_test.cpp

complete_test.o_dbg:	version.h defs.h config.h compile.h types.h result.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h complete_test.h complete_test.cpp
	g++ -Wall -Wextra -pedantic -Wno-long-long -Wno-sign-compare -g -c -ocomplete_test.o_dbg complete_test.cpp

optimizer_main.o_dbg:	version.h defs.h config.h compile.h types.h result.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h optimizer_main.h optimizer_main.cpp
	g++ -Wall -Wextra -pedantic -Wno-long-long -Wno-sign-compare -g -c -ooptimizer_main.o_dbg optimizer_main.cpp

autooptim_main.o_dbg:	version.h defs.h config.h compile.h types.h result.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h autooptim_main.h autooptim_main.cpp
	g++ -Wall -Wextra -pedantic -Wno-long-long -Wno-sign-compare -g -c -oautooptim_main.o_dbg autooptim_main.cpp

resolver_main.o_dbg:	version.h defs.h config.h compile.h types.h result.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h resolver_main.h resolver_main.cpp
	g++ -Wall -Wextra -pedantic -Wno-long-long -Wno-sign-compare -g -c -oresolver_main.o_dbg resolver_main.cpp

solver_main.o_dbg:	version.h defs.h config.h compile.h types.h result.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h solver_main.h solver_main.cpp
	g++ -Wall -Wextra -pedantic -Wno-long-long -Wno-sign-compare -g -c -osolver_main.o_dbg solver_main.cpp

gridgen_main.o_dbg:	version.h defs.h config.h compile.h types.h result.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h gridgen_main.h gridgen_main.cpp
	g++ -Wall -Wextra -pedantic -Wno-long-long -Wno-sign-compare -g -c -ogridgen_main.o_dbg gridgen_main.cpp

netgen_main.o_dbg:	version.h defs.h config.h compile.h types.h result.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h netgen_main.h netgen_main.cpp
	g++ -Wall -Wextra -pedantic -Wno-long-long -Wno-sign-compare -g -c -onetgen_main.o_dbg netgen_main.cpp

hypergen_main.o_dbg:	version.h defs.h config.h compile.h types.h result.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h hypergen_main.h hypergen_main.cpp
	g++ -Wall -Wextra -pedantic -Wno-long-long -Wno-sign-compare -g -c -ohypergen_main.o_dbg hypergen_main.cpp

simplify_main.o_dbg:	version.h defs.h config.h compile.h types.h result.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h simplify_main.h simplify_main.cpp
	g++ -Wall -Wextra -pedantic -Wno-long-long -Wno-sign-compare -g -c -osimplify_main.o_dbg simplify_main.cpp


types.o_opt:	version.h defs.h config.h compile.h types.h result.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h reloc_test.h types.cpp
	g++ -Wall -Wextra -pedantic -Wno-long-long -Wno-sign-compare -c -O3 -mtune=native -otypes.o_opt types.cpp

reloc.o_opt:	version.h defs.h config.h compile.h types.h result.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h reloc_test.h reloc.cpp
	g++ -Wall -Wextra -pedantic -Wno-long-long -Wno-sign-compare -c -O3 -mtune=native -oreloc.o_opt reloc.cpp

hierarch.o_opt:	version.h defs.h config.h compile.h types.h result.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h reloc_test.h hierarch.cpp
	g++ -Wall -Wextra -pedantic -Wno-long-long -Wno-sign-compare -c -O3 -mtune=native -ohierarch.o_opt hierarch.cpp

multirobot.o_opt:	version.h defs.h config.h compile.h types.h result.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h reloc_test.h multirobot.cpp
	g++ -Wall -Wextra -pedantic -Wno-long-long -Wno-sign-compare -c -O3 -mtune=native -omultirobot.o_opt multirobot.cpp

search.o_opt:	version.h defs.h config.h compile.h types.h result.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h reloc_test.h search.cpp
	g++ -Wall -Wextra -pedantic -Wno-long-long -Wno-sign-compare -c -O3 -mtune=native -osearch.o_opt search.cpp

cnf.o_opt:	version.h defs.h config.h compile.h types.h result.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h reloc_test.h cnf.cpp
	g++ -Wall -Wextra -pedantic -Wno-long-long -Wno-sign-compare -c -O3 -mtune=native -ocnf.o_opt cnf.cpp

compress.o_opt:	version.h defs.h config.h compile.h types.h result.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h reloc_test.h compress.cpp
	g++ -Wall -Wextra -pedantic -Wno-long-long -Wno-sign-compare -c -O3 -mtune=native -ocompress.o_opt compress.cpp

statistics.o_opt:	version.h defs.h config.h compile.h types.h result.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h reloc_test.h statistics.cpp
	g++ -Wall -Wextra -pedantic -Wno-long-long -Wno-sign-compare -c -O3 -mtune=native -ostatistics.o_opt statistics.cpp

graph.o_opt:	version.h defs.h config.h compile.h types.h result.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h reloc_test.h graph.cpp
	g++ -Wall -Wextra -pedantic -Wno-long-long -Wno-sign-compare -c -O3 -mtune=native -ograph.o_opt graph.cpp

complete.o_opt:	version.h defs.h config.h compile.h types.h result.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h reloc_test.h complete.cpp
	g++ -Wall -Wextra -pedantic -Wno-long-long -Wno-sign-compare -c -O3 -mtune=native -ocomplete.o_opt complete.cpp

reloc_test.o_opt:	version.h defs.h config.h compile.h types.h result.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h reloc_test.h reloc_test.cpp
	g++ -Wall -Wextra -pedantic -Wno-long-long -Wno-sign-compare -c -O3 -mtune=native -oreloc_test.o_opt reloc_test.cpp

multirobot_test.o_opt:	version.h defs.h config.h compile.h types.h result.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h graph.h complete.h statistics.h multirobot_test.h multirobot_test.cpp
	g++ -Wall -Wextra -pedantic -Wno-long-long -Wno-sign-compare -c -O3 -mtune=native -omultirobot_test.o_opt multirobot_test.cpp

hierarch_test.o_opt:	version.h defs.h config.h compile.h types.h result.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h hierarch_test.h hierarch_test.cpp
	g++ -Wall -Wextra -pedantic -Wno-long-long -Wno-sign-compare -c -O3 -mtune=native -ohierarch_test.o_opt hierarch_test.cpp

cnf_test.o_opt:	version.h defs.h config.h compile.h types.h result.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h cnf_test.h cnf_test.cpp
	g++ -Wall -Wextra -pedantic -Wno-long-long -Wno-sign-compare -c -O3 -mtune=native -ocnf_test.o_opt cnf_test.cpp

alldiff_test.o_opt:	version.h defs.h config.h compile.h types.h result.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h alldiff_test.h alldiff_test.cpp
	g++ -Wall -Wextra -pedantic -Wno-long-long -Wno-sign-compare -c -O3 -mtune=native -oalldiff_test.o_opt alldiff_test.cpp

compress_test.o_opt:	version.h defs.h config.h compile.h types.h result.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h compress_test.h compress_test.cpp
	g++ -Wall -Wextra -pedantic -Wno-long-long -Wno-sign-compare -c -O3 -mtune=native -ocompress_test.o_opt compress_test.cpp

complete_test.o_opt:	version.h defs.h config.h compile.h types.h result.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h complete_test.h complete_test.cpp
	g++ -Wall -Wextra -pedantic -Wno-long-long -Wno-sign-compare -c -O3 -mtune=native -ocomplete_test.o_opt complete_test.cpp

optimizer_main.o_opt:	version.h defs.h config.h compile.h types.h result.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h optimizer_main.h optimizer_main.cpp
	g++ -Wall -Wextra -pedantic -Wno-long-long -Wno-sign-compare -c -O3 -mtune=native -ooptimizer_main.o_opt optimizer_main.cpp

autooptim_main.o_opt:	version.h defs.h config.h compile.h types.h result.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h autooptim_main.h autooptim_main.cpp
	g++ -Wall -Wextra -pedantic -Wno-long-long -Wno-sign-compare -c -O3 -mtune=native -oautooptim_main.o_opt autooptim_main.cpp

resolver_main.o_opt:	version.h defs.h config.h compile.h types.h result.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h resolver_main.h resolver_main.cpp
	g++ -Wall -Wextra -pedantic -Wno-long-long -Wno-sign-compare -c -O3 -mtune=native -oresolver_main.o_opt resolver_main.cpp

solver_main.o_opt:	version.h defs.h config.h compile.h types.h result.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h solver_main.h solver_main.cpp
	g++ -Wall -Wextra -pedantic -Wno-long-long -Wno-sign-compare -c -O3 -mtune=native -osolver_main.o_opt solver_main.cpp

gridgen_main.o_opt:	version.h defs.h config.h compile.h types.h result.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h gridgen_main.h gridgen_main.cpp
	g++ -Wall -Wextra -pedantic -Wno-long-long -Wno-sign-compare -c -O3 -mtune=native -ogridgen_main.o_opt gridgen_main.cpp

netgen_main.o_opt:	version.h defs.h config.h compile.h types.h result.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h netgen_main.h netgen_main.cpp
	g++ -Wall -Wextra -pedantic -Wno-long-long -Wno-sign-compare -c -O3 -mtune=native -onetgen_main.o_opt netgen_main.cpp

hypergen_main.o_opt:	version.h defs.h config.h compile.h types.h result.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h hypergen_main.h hypergen_main.cpp
	g++ -Wall -Wextra -pedantic -Wno-long-long -Wno-sign-compare -c -O3 -mtune=native -ohypergen_main.o_opt hypergen_main.cpp

simplify_main.o_opt:	version.h defs.h config.h compile.h types.h result.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h simplify_main.h simplify_main.cpp
	g++ -Wall -Wextra -pedantic -Wno-long-long -Wno-sign-compare -c -O3 -mtune=native -osimplify_main.o_opt simplify_main.cpp


clean:
	rm -f *~ *.o_dbg *.o_opt *.bak output.txt *.cnf *.pddl compress/* *.out *.rbt simplify/* test_basic_reLOC test_multirobot_reLOC test_hierarch_reLOC test_cnf_reLOC test_alldiff_reLOC test_compress_reLOC test_complete_reLOC optimizer_reLOC autooptim_reLOC resolver_reLOC solver_reLOC gridgen_reLOC netgen_reLOC hypergen_reLOC simplify_reLOC
