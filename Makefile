CFLAGS += -Wall -O2 

WRT54GMEMOBJS = tjtag.o

all: tjtag

wrt54g: $(WRT54GMEMOBJS)
	gcc $(CFLAGS) -o $@ $(WRT54GMEMOBJS)

clean:
	rm -rf *.o tjtag
