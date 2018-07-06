LIBS=-L ../DynamixelSDK/c++/build/linux_sbc -ldxl_sbc_cpp
LIBS=

go: main.o BioloidEX.o ax12Serial.o
	g++ -std=c++14 $^ -o go -O0 -g $(LIBS) -lrt 2>&1

%.o: %.cpp
	g++ -std=c++14 $^ -I .  -I ../DynamixelSDK/c++/include/dynamixel_sdk -c -O0 -g 2>&1

clean:
	rm *.o
