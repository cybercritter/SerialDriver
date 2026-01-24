CC = gcc
AR = ar
CFLAGS = -Wall -Wextra 
LDFLAGS =
SRC = $(wildcard src/*.c)
OBJ = $(patsubst src/%.c,$(OBJDIR)/%.o,$(SRC))
LIB = serialdriverlib.a
INC= -Iinc

.PHONY: all clean mkdir

OUTDIR = bin
OBJDIR = obj

all: mkdir $(LIB)
$(LIB): $(OBJ)
	$(AR) rcs $(OUTDIR)/$@ $^

$(OBJDIR)/%.o: src/%.c
	$(CC) $(CFLAGS) $(INC) -c $< -o $@

mkdir:
	mkdir -p $(OBJDIR) $(OUTDIR) $(LIBDIR)
clean:
	rm -rf $(OBJDIR) $(OUTDIR) $(LIBDIR) $(LIB)
