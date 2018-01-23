# www.cs.colby.edu/maxwell/courses/tutorials/maketutor

CC 	= gcc
CFLAGS 	= -Wall -O3 -ggdb -I. -lpthread -lwiringPi
DEPS 	= ~/projects/gnuplot/gnuplot_i.h
OBJS 	= ~/projects/gnuplot/gnuplot_i.o P4.o

%.o:	%.c  $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

P4:	$(OBJS)
	$(CC)    -o $@ $^ $(CFLAGS)
