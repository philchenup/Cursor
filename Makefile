CXX ?= g++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -Werror -Iinclude -Ithird_party

EXAMPLE := place_config_example
EXAMPLE_SRC := src/PlaceConfigJsonExample.cpp

.PHONY: all example test clean

all: example

example: $(EXAMPLE)

$(EXAMPLE): $(EXAMPLE_SRC) include/PlaceConfig.h include/PlaceConfigJson.h
	$(CXX) $(CXXFLAGS) $(EXAMPLE_SRC) -o $@

test: $(EXAMPLE)
	./$(EXAMPLE) /tmp/place_config.json

clean:
	rm -f $(EXAMPLE) place_config.json
