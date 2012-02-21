
SRCDIR  = src
OBJDIR  = out

CC	 = gcc
CXX 	 = g++
CFLAGS   = -g -c -O2 -Wall -D_GNU_SOURCE
CXXFLAGS = -g -c -O2 -Wall -D_GNU_SOURCE
LDFLAGS  =
INCLUDES = -I./src -I.
LIBS     = -lrt -lm -lpthread
#-lstdc++

SRCS	= 	main.c		\
		mem.c		\
		shm.c		\
		spi.c		\
		udp.c       \
        crc.c		\
        w5d.c


OBJS	= $(patsubst %.c,$(OBJDIR)/%.o,$(SRCS))

all: $(OBJDIR) w5d-overo

w5d-overo: $(OBJS)
	$(CC) $(OBJS) $(LDFLAGS) $(LIBS) -o $@

$(OBJDIR)/%.o: $(SRCDIR)/%.c
	$(CC) $(INCLUDES) -o $@ $< $(CFLAGS)

$(OBJDIR)/%.o: $(SRCDIR)/%.cpp
	$(CXX) $(INCLUDES) -o $@ $< $(CXXFLAGS)

$(OBJDIR):
	install -d $(OBJDIR)

.PHONY: clean

clean:
	$(RM) $(OBJS) w5d-overo


#$@ name of the target
#$^ name of all prerequisites with duplicates removed
#$< name of the first prerequisite

