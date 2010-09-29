.SUFFIXES: .cpp

CC = g++
# CFLAGS  = -Wall -O2
INCLUDE_DIR = ../include/
MOTION_PLANNER_LIB_DIR = ../lib/
CFLAGS  = -Wall -I$(INCLUDE_DIR) -shared
LDLIBS  = -lm -lPQP -lemon -L$(MOTION_PLANNER_LIB_DIR) -lMotionPlanner

SRCS    = p3at.cpp

OBJECTS = p3at.o

TARGET = p3at.out

CLEAN   = $(OBJECTS) $(TARGET)

$(TARGET): $(OBJECTS)
	$(CC) $(OBJECTS) -o $(TARGET) $(LDLIBS)

.cpp.o:
	$(CC) -shared $(CFLAGS) -c $<

run: $(TARGET)
	$(TARGET)

clean:
	/bin/rm -f $(CLEAN)

