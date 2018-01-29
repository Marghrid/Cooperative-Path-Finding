FLAGS = -Wall -std=c++11
INC   = -I src/ -I include/glucose/
LIB   = -L include/glucose/simp

all: directories build/graph.o build/instance.o build/parser.o build/simplified_encoder.o
	g++ $(FLAGS) $(INC) $(LIB) -o bin/a.out build/graph.o build/instance.o build/parser.o build/simplified_encoder.o -l_standard src/main.cpp 

directories:
	mkdir -p build 
	mkdir -p bin

build/graph.o: src/graph.cpp src/graph.h
	g++ $(FLAGS) -c $(INC) -o build/graph.o src/graph.cpp

build/instance.o: src/instance.cpp src/instance.h src/agent.h src/graph.h
	g++ $(FLAGS) -c $(INC) -o build/instance.o src/instance.cpp

build/parser.o: src/parser.cpp src/parser.h src/instance.h
	g++ $(FLAGS) -c $(INC) -o build/parser.o src/parser.cpp

build/simplified_encoder.o: src/simplified_encoder.cpp src/simplified_encoder.h
	g++ $(FLAGS) -c $(INC) -o build/simplified_encoder.o src/simplified_encoder.cpp

clean:
	rm -f build/*.o
	rm -f bin/*
	rm -r build
	rm -r bin
