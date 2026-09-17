CXX ?= g++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -Werror -Iinclude
LDFLAGS ?=

BUILD_DIR := build
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

LIB_SRC := src/KukaWeldComm.cpp
TEST_SRC := src/KukaWeldCommTest.cpp
EXAMPLE_SRC := src/KukaWeldCommExample.cpp

.PHONY: all test example docs clean

all: test example docs

$(BUILD_DIR)/KukaWeldComm.o: $(LIB_SRC) include/KukaWeldComm.h | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -c $(LIB_SRC) -o $@

$(BUILD_DIR)/kuka_weld_comm_test: $(BUILD_DIR)/KukaWeldComm.o $(TEST_SRC)
	$(CXX) $(CXXFLAGS) $(TEST_SRC) $(BUILD_DIR)/KukaWeldComm.o -o $@ $(LDFLAGS)

$(BUILD_DIR)/kuka_weld_comm_example: $(BUILD_DIR)/KukaWeldComm.o $(EXAMPLE_SRC)
	$(CXX) $(CXXFLAGS) $(EXAMPLE_SRC) $(BUILD_DIR)/KukaWeldComm.o -o $@ $(LDFLAGS)

test: $(BUILD_DIR)/kuka_weld_comm_test
	$(BUILD_DIR)/kuka_weld_comm_test

example: $(BUILD_DIR)/kuka_weld_comm_example

docs: example
	mkdir -p docs kuka/EthernetKRL
	$(BUILD_DIR)/kuka_weld_comm_example docs/kuka_weld_comm_table.csv kuka/EthernetKRL/WeldHost.xml > docs/kuka_weld_comm_table.md
	@echo "wrote docs/kuka_weld_comm_table.csv docs/kuka_weld_comm_table.md kuka/EthernetKRL/WeldHost.xml"

clean:
	rm -rf $(BUILD_DIR)
