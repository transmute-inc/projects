# www.cs.colby.edu/maxwell/courses/tutorials/maketutor

CC 	= gcc
CFLAGS 	= -Wall -O3 -ggdb -I. -lpthread -lwiringPi
OBJS 	=  test.o

%.o:	%.c 
	$(CC) -c -o $@ $< $(CFLAGS)

test:	$(OBJS)
	$(CC)    -o $@ $^ $(CFLAGS)
