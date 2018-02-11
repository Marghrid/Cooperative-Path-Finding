FLAGS = -Wall -std=c++11
INC   = -I src/ -I include/glucose/ -I src/searches -I src/encoders
LIB   = -L include/glucose/simp

all: directories build/Graph.o build/Instance.o build/Parser.o build/Simplified_solver.o
	g++ $(FLAGS) $(INC) $(LIB) -o bin/a.out build/Graph.o build/Instance.o build/Parser.o build/Simplified_solver.o -l_standard src/main.cpp 

directories:
	mkdir -p build 
	mkdir -p bin

build/Graph.o: src/Graph.cpp src/Graph.h
	g++ $(FLAGS) -c $(INC) -o build/Graph.o src/Graph.cpp

build/Instance.o: src/Instance.cpp src/Instance.h src/Agent.h src/Graph.h
	g++ $(FLAGS) -c $(INC) -o build/Instance.o src/Instance.cpp

build/Parser.o: src/Parser.cpp src/Parser.h src/Instance.h
	g++ $(FLAGS) -c $(INC) -o build/Parser.o src/Parser.cpp

build/Simplified_solver.o: src/Simplified_solver.cpp src/Simplified_solver.h
	g++ $(FLAGS) -c $(INC) -o build/Simplified_solver.o src/Simplified_solver.cpp

clean:
	rm -f build/*.o
	rm -f bin/*
	rm -r build
	rm -r bin
