all:
	g++ -L/usr/local/lib -std=c++11 PathCalculator.cpp main.cpp -o rigid.o -lompl
clean:
	rm -rf rigid.o