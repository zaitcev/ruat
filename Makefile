##

LDFLAGS = -lm -lpthread

CFLAGS += $(shell pkg-config --cflags librtlsdr)
LDFLAGS += $(shell pkg-config --libs librtlsdr)

all: ruat

ruat: ruat.o

ruat.o: ruat.c

tester: tester.o fec.o

tester.o: tester.c fec.h

fec.o: fec.h fec.c

check: tester
	./tester

clean:
	rm -f ruat tester *.o
