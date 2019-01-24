# www.cs.colby.edu/maxwell/courses/tutorials/maketutor
# CFLAGS 	= -Wall -O3 -ggdb -I. -lpthread -lpigpio -lrt

CC 	= gcc
CFLAGS 	= -Wall -ggdb -pthread -I. -lpigpio -lrt
DEPS 	= ~/projects/gnuplot/gnuplot_i.h ~/projects/P7/max2871.h
OBJS 	= ~/projects/gnuplot/gnuplot_i.o p7.o

%.o:	%.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

p7:	$(OBJS)
	$(CC)    -o $@ $^ $(CFLAGS)
