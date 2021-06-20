# dragino lora testing
# Single lora testing app

CC=g++
CFLAGS=-c -Wall -I include/
LIBS=-lwiringPi

all: Receiver

Receiver: receiver.o
	$(CC) receiver.o  $(LIBS) -o Receiver

receiver.o: receiver.c
	$(CC) $(CFLAGS) receiver.c

clean:
	rm *.o dragino_lora_app
	rm *.o myApp
	rm *.o Receiver
