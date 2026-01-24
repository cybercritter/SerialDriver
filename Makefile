CC = gcc
AR = ar
CFLAGS = -Wall -Wextra 
LDFLAGS =
SRC = $(wildcard src/*.c)
OBJ = $(patsubst src/%.c,$(OBJDIR)/%.o,$(SRC))
LIB = libserialdriver.a
LIBDIR = lib
LIBNAME = serialdriver
INC= -Iinc

.PHONY: all clean mkdir

OUTDIR = bin
OBJDIR = obj

all: mkdir $(LIB) build_demo
$(LIB): $(OBJ)
	$(AR) rcs $(LIBDIR)/$@ $^

$(OBJDIR)/%.o: src/%.c
	$(CC) $(CFLAGS) $(INC) -c $< -o $@

build_demo:
	$(CC) $(CFLAGS) $(INC) $(LDFLAGS) -L$(LIBDIR) -l$(LIBNAME) -o $(OUTDIR)/demo_serial_driver demo/demo_serial_driver.c
mkdir:
	mkdir -p $(OBJDIR) $(OUTDIR) $(LIBDIR)
clean:
	rm -rf $(OBJDIR) $(OUTDIR) $(LIBDIR) $(LIB)
