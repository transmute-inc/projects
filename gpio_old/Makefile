# www.cs.colby.edu/maxwell/courses/tutorials/maketutor
# CFLAGS 	= -Wall -O3 -ggdb -I. -lpthread -lpigpio -lrt

CC 	= gcc
CFLAGS 	= -Wall -pthread -I. -lpigpio -lrt
OBJS 	=  test.o

%.o:	%.c 
	$(CC) -c -o $@ $< $(CFLAGS)

test:	$(OBJS)
	$(CC)    -o $@ $^ $(CFLAGS)
