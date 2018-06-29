all:
	g++ main.cpp -lglut -lGL -lGLU  -o main -fpermissive --std=c++11
	./main
