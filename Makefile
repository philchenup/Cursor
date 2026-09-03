CXX ?= g++
QT_CFLAGS := $(shell pkg-config --cflags Qt5Core)
QT_LIBS := $(shell pkg-config --libs Qt5Core)
CXXFLAGS ?= -std=c++17 -Wall -Wextra -Werror -fPIC -Iinclude $(QT_CFLAGS)

EXAMPLE := format_comm_example
TEST := format_comm_test

.PHONY: all example test clean

all: example test

example: $(EXAMPLE)

test: $(TEST)
	./$(TEST)

$(EXAMPLE): src/CommFormatExample.cpp src/CommFormat.cpp include/CommFormat.h
	$(CXX) $(CXXFLAGS) src/CommFormatExample.cpp src/CommFormat.cpp $(QT_LIBS) -o $@

$(TEST): src/CommFormatTest.cpp src/CommFormat.cpp include/CommFormat.h
	$(CXX) $(CXXFLAGS) src/CommFormatTest.cpp src/CommFormat.cpp $(QT_LIBS) -o $@

clean:
	rm -f $(EXAMPLE) $(TEST)
