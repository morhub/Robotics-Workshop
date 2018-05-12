all:
	g++ -L/usr/local/lib -std=c++11 -g PathCalculator.cpp main.cpp -o rigid.o -lompl `pkg-config --cflags --libs opencv`
clean:
	rm -rf rigid.o
