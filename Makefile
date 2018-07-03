go: main.o BioloidEX.o ax12Serial.o
	g++ -std=c++14 $^ -o go -O0 -g 2>&1

%.o: %.cpp
	g++ -std=c++14 $^ -I . -c -O0 -g 2>&1

clean:
	rm *.o
