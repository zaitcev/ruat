##
## ruat
##

CFLAGS = -Wall -O2
LDFLAGS =
#CFLAGS = -Wall -g -pg
#LDFLAGS = -pg
LIBS = -lm -lpthread

CFLAGS += $(shell pkg-config --cflags librtlsdr)
#LDFLAGS +=
LIBS_R = $(LIBS) $(shell pkg-config --libs librtlsdr)

# Although libairspy installs libairspy.pc, the default location is in
# /usr/local and thus not being found by pkg-config. So, might as well
# just hardcode this.
CFLAGS += -I/usr/local/include/libairspy
LDFLAGS += -L/usr/local/lib
LIBS_A = $(LIBS) -lairspy

all: ruat ruat_airspy tester

ruat: ruat.o fec.o
	${CC} ${LDFLAGS} -o ruat ruat.o fec.o ${LIBS_R}

ruat.o: ruat.c

ruat_airspy: ruat_airspy.o fec.o
	${CC} ${LDFLAGS} -o ruat_airspy ruat_airspy.o fec.o ${LIBS_A}

ruat_airspy.o: ruat_airspy.c

tester: tester.o fec.o

tester.o: tester.c fec.h

fec.o: fec.h fec.c

check: tester
	./tester

clean:
	rm -f ruat ruat_airspy tester *.o
