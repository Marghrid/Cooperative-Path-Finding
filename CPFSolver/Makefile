GLUCOSE_DIR ?= ../glucose

FLAGS = -Wall -std=c++14 -g
INC   = -I src/ -I $(GLUCOSE_DIR) -I src/searches/ -I src/encoders/ -I src/exceptions
LIB   = -L $(GLUCOSE_DIR)/simp

all: directories build/Graph.o build/Instance.o build/Solution.o build/Parser.o build/SimplifiedEncoder.o build/CPFSolver.o
	g++ $(FLAGS) $(INC) $(LIB) -o bin/CPFSolver build/Graph.o build/Instance.o build/Solution.o build/Parser.o build/CPFSolver.o build/SimplifiedEncoder.o  src/main.cpp -l_standard

directories:
	mkdir -p build 
	mkdir -p bin

build/Graph.o: src/Graph.cpp src/Graph.h
	g++ $(FLAGS) -c $(INC) -o build/Graph.o src/Graph.cpp

build/Instance.o: src/Instance.cpp src/Instance.h src/Agent.h src/Graph.h
	g++ $(FLAGS) -c $(INC) -o build/Instance.o src/Instance.cpp

build/Solution.o: src/Solution.cpp src/Solution.h src/Instance.h
	g++ $(FLAGS) -c $(INC) -o build/Solution.o src/Solution.cpp

build/Parser.o: src/Parser.cpp src/Parser.h src/Instance.h
	g++ $(FLAGS) -c $(INC) -o build/Parser.o src/Parser.cpp

build/SimplifiedEncoder.o: src/encoders/SimplifiedEncoder.cpp src/encoders/SimplifiedEncoder.h src/Encoder.h src/Instance.h src/Solution.h
	g++ $(FLAGS) -c $(INC) -o build/SimplifiedEncoder.o src/encoders/SimplifiedEncoder.cpp

build/CPFSolver.o: src/CPFSolver.cpp src/CPFSolver.h src/Instance.h src/Solution.h src/Parser.h src/Encoder.h src/Search.h src/encoders/SimplifiedEncoder.h src/searches/SAT_UNSATSearch.h src/searches/UNSAT_SATSearch.h src/searches/BinarySearch.h
	g++ $(FLAGS) -c $(INC) -o build/CPFSolver.o src/CPFSolver.cpp

clean:
	rm -f build/*.o
	rm -f bin/*
	rm -r build
	rm -r bin
