FLAGS=-Wall -std=c++11

all: directories build/graph.o build/instance.o build/parser.o build/direct_encoder.o
	g++ $(FLAGS) -o bin/a.out build/graph.o build/instance.o build/parser.o build/direct_encoder.o src/main.cpp

directories:
	mkdir -p build
	mkdir -p bin

build/graph.o: src/graph.cpp src/graph.h
	g++ $(FLAGS) -c -o build/graph.o src/graph.cpp

build/instance.o: src/instance.cpp src/instance.h src/agent.h src/graph.h
	g++ $(FLAGS) -c -o build/instance.o src/instance.cpp

build/parser.o: src/parser.cpp src/parser.h src/instance.h
	g++ $(FLAGS) -c -o build/parser.o src/parser.cpp

build/direct_encoder.o: src/direct_encoder.cpp src/direct_encoder.h src/cnf.h
	g++ $(FLAGS) -c -o build/direct_encoder.o src/direct_encoder.cpp

clean:
	rm -f build/*.o
	rm -f bin/*
	rm -r build
	rm -r bin
