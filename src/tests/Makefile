.SUFFIXES: .cpp

CC = g++
INCLUDE_DIR = ../include/
CFLAGS  = -Wall -I$(INCLUDE_DIR)
LDLIBS  = -lm -lPQP -lemon

SRCS    = p3at.cpp

MY_MOTION_PLANNER_LIB = ../lib/libMotionPlanner.a

OBJECTS = p3at.o

TARGET = p3at.out

CLEAN   = $(OBJECTS) $(TARGET)

$(TARGET): $(OBJECTS) $(MY_MOTION_PLANNER_LIB)
	$(CC) $(OBJECTS) $(MY_MOTION_PLANNER_LIB) -o $(TARGET) $(LDLIBS)

.cpp.o:
	$(CC) $(CFLAGS) -c $<

run: $(TARGET)
	$(TARGET)

clean:
	@rm $(CLEAN)
