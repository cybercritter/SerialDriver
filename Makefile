CC = gcc
CXX = g++
AR = ar
CFLAGS = -Wall -Wextra -g -std=c99
CXXFLAGS = -Wall -Wextra -g -std=c++17
LDFLAGS =
SRC = $(wildcard src/*.c)
OBJ = $(patsubst src/%.c,$(OBJDIR)/%.o,$(SRC))
TEST_SRC = $(wildcard tests/*.cpp)
TEST_BIN = $(OUTDIR)/serialdriver_tests
GTEST_DIR = third_party/googletest
GTEST_INC = -I$(GTEST_DIR)/googletest/include -I$(GTEST_DIR)/googletest
GTEST_SRC = $(GTEST_DIR)/googletest/src/gtest-all.cc
GTEST_MAIN_SRC = $(GTEST_DIR)/googletest/src/gtest_main.cc
GTEST_OBJ = $(OBJDIR)/gtest-all.o
GTEST_MAIN_OBJ = $(OBJDIR)/gtest_main.o
LIB = libserialdriver.a
INC= -Iinc

.PHONY: all clean mkdir demo test

OUTDIR = bin
OBJDIR = obj

all: mkdir $(LIB) demo run_demo_tests
$(LIB): $(OBJ)
	$(AR) rcs $@ $^

demo: $(LIB)
	$(CC) $(CFLAGS) $(INC) -L. -lserialdriver -o $(OUTDIR)/demo demo/demo_serial_driver.c

run_demo_tests: test
	$(OUTDIR)/serialdriver_tests


test: mkdir $(LIB) $(TEST_SRC) $(GTEST_OBJ) $(GTEST_MAIN_OBJ)
	$(CXX) $(CXXFLAGS) $(INC) $(GTEST_INC) -L. -lserialdriver -pthread \
		-o $(TEST_BIN) $(TEST_SRC) $(GTEST_OBJ) $(GTEST_MAIN_OBJ)

$(GTEST_OBJ): $(GTEST_SRC)
	$(CXX) $(CXXFLAGS) $(GTEST_INC) -c $< -o $@

$(GTEST_MAIN_OBJ): $(GTEST_MAIN_SRC)
	$(CXX) $(CXXFLAGS) $(GTEST_INC) -c $< -o $@

$(OBJDIR)/%.o: src/%.c
	$(CC) $(CFLAGS) $(INC) -c $< -o $@

mkdir:
	mkdir -p $(OBJDIR) $(OUTDIR)
clean:
	rm -rf $(OBJDIR) $(OUTDIR) $(LIB)
