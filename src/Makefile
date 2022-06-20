CXX=$(CROSS_COMPILE)g++
RM=rm
MKDIR_P=mkdir -p
OUT_DIR=../build/
INCLUDE =  -I/root/eigen
INCLUDE +=  -I/opt/redpitaya/include
INCLUDE += -I../include
LIBS = -L/opt/redpitaya/lib

CXXFLAGS+= -std=c++14 $(INCLUDE)
CXXFLAGS+= -O3
LDFLAGS = $(LIBS)
LDFLAGS+= -lrp


all : prog01 prog02

prog01 : main.cpp
	$(MKDIR_P) $(OUT_DIR)
	$(CXX) -o $(OUT_DIR)kfm_controller main.cpp $(LDFLAGS) $(CXXFLAGS)

prog02 : parameter_readout.cpp
	$(CXX) -o $(OUT_DIR)parameter_readout parameter_readout.cpp $(LDFLAGS) $(CXXFLAGS)
