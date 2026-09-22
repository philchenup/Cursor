CXX ?= g++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -Werror -Iinclude
LDFLAGS ?=

BUILD_DIR := build
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

WELD_LIB := src/KukaWeldComm.cpp
WELD_TEST := src/KukaWeldCommTest.cpp
WELD_EXAMPLE := src/KukaWeldCommExample.cpp
PN_LIB := src/KukaProfinetIo.cpp
PN_TEST := src/KukaProfinetIoTest.cpp
PN_EXAMPLE := src/KukaProfinetIoExample.cpp

.PHONY: all test example docs clean

all: test example docs

$(BUILD_DIR)/KukaWeldComm.o: $(WELD_LIB) include/KukaWeldComm.h | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -c $(WELD_LIB) -o $@

$(BUILD_DIR)/KukaProfinetIo.o: $(PN_LIB) include/KukaProfinetIo.h | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -c $(PN_LIB) -o $@

$(BUILD_DIR)/kuka_weld_comm_test: $(BUILD_DIR)/KukaWeldComm.o $(WELD_TEST)
	$(CXX) $(CXXFLAGS) $(WELD_TEST) $(BUILD_DIR)/KukaWeldComm.o -o $@ $(LDFLAGS)

$(BUILD_DIR)/kuka_pn_io_test: $(BUILD_DIR)/KukaProfinetIo.o $(PN_TEST)
	$(CXX) $(CXXFLAGS) $(PN_TEST) $(BUILD_DIR)/KukaProfinetIo.o -o $@ $(LDFLAGS)

$(BUILD_DIR)/kuka_weld_comm_example: $(BUILD_DIR)/KukaWeldComm.o $(WELD_EXAMPLE)
	$(CXX) $(CXXFLAGS) $(WELD_EXAMPLE) $(BUILD_DIR)/KukaWeldComm.o -o $@ $(LDFLAGS)

$(BUILD_DIR)/kuka_pn_io_example: $(BUILD_DIR)/KukaProfinetIo.o $(PN_EXAMPLE)
	$(CXX) $(CXXFLAGS) $(PN_EXAMPLE) $(BUILD_DIR)/KukaProfinetIo.o -o $@ $(LDFLAGS)

test: $(BUILD_DIR)/kuka_weld_comm_test $(BUILD_DIR)/kuka_pn_io_test
	$(BUILD_DIR)/kuka_weld_comm_test
	$(BUILD_DIR)/kuka_pn_io_test

example: $(BUILD_DIR)/kuka_weld_comm_example $(BUILD_DIR)/kuka_pn_io_example

docs: example
	mkdir -p docs kuka/EthernetKRL
	$(BUILD_DIR)/kuka_weld_comm_example docs/kuka_weld_comm_table.csv kuka/EthernetKRL/WeldHost.xml > docs/kuka_weld_comm_table.md
	$(BUILD_DIR)/kuka_pn_io_example docs/kuka_profinet_io_map.csv > /tmp/kuka_pn_io_map.md
	@echo "wrote docs/kuka_weld_comm_table.csv docs/kuka_weld_comm_table.md kuka/EthernetKRL/WeldHost.xml docs/kuka_profinet_io_map.csv"

clean:
	rm -rf $(BUILD_DIR)
