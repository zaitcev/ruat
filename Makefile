##

LDFLAGS = -lm -lpthread

CFLAGS += $(shell pkg-config --cflags librtlsdr)
LDFLAGS += $(shell pkg-config --libs librtlsdr)

all: ruat

ruat: ruat.o

ruat.o: ruat.c

#check: ruat
#	cd test && nosetests --exe

clean:
	rm -f ruat *.o
