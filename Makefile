all:
	g++ main.cpp -lglut -lGL -lGLU -O3 -o main -fpermissive --std=c++11
	./main
