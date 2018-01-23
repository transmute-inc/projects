# www.cs.colby.edu/maxwell/courses/tutorials/maketutor

CC 	= gcc
CFLAGS 	= -Wall -O3 -ggdb -I. -lpthread -lwiringPi
OBJS 	=  P1.o

%.o:	%.c 
	$(CC) -c -o $@ $< $(CFLAGS)

P1:	$(OBJS)
	$(CC)    -o $@ $^ $(CFLAGS)
