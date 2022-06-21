#################################################################################

RM = rm -vf

DEBUG	= -O2 -g
CC	= gcc
CXX	= g++

INCLUDE	 = -I/usr/local/include
INCLUDE += -I ../../Tools/include -I../../adc-dac

CFLAGS   = $(DEBUG) $(INCLUDE) -Wall -Winline -pipe
CXXFLAGS = $(DEBUG) -Wall -fmessage-length=0 -std=c++20 $(INCLUDE)

OBJS= I2Cdev.o ADS1115.o LightTracker.o

LDFLAGS	= -L/usr/local/lib
LDLIBS  = -lpigpio -lm #-lpthread -lrt

TARGET  = LightTracker

#################################################################################

all: $(TARGET)

LightTracker: $(OBJS) *.cpp *.h  Makefile # LightTracker.cpp $(OBJS)
	@echo [$<]
	$(CXX) $(CXXFLAGS) -o $@ $(OBJS) $(LDLIBS)

#################################################################################

.c.o:
	@echo [CC] $<
	$(CC) -c $(CFLAGS) $< -o $@

.cpp.o:
	@echo [CC] $<
	$(CXX) -c $(CXXFLAGS) $< -o $@

echo:
	@echo "[LDLIBS]\n  $(LDLIBS)"
	@echo "[CXX_OBJS]\n  $(OBJS)"
	@echo "[BINS]\n  $(BINS)"

clean:
	@echo "[Clean]"
	$(RM) $(OBJ) *.o *~ core tags

clobber: clean
	@echo "[Clobber]"
	$(RM) $(TARGET)

#################################################################################
