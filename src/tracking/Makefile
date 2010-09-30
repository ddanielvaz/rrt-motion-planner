.SUFFIXES: .cpp .cxx

CC = g++
INCLUDE_DIR = ../include
CFLAGS  = -Wall -I$(INCLUDE_DIR) `pkg-config --cflags opencv playerc++`
LDLIBS  = -lm -lemon `pkg-config --libs playerc++`

SRCS    = tracking.cpp PlayerTracking.cxx RobotProxy.cxx Position2DProxy.cxx

OBJECTS = Position2DProxy.o RobotProxy.o PlayerTracking.o tracking.o

MY_LIBS = ../lib/MathFunctions.o

TARGET = tracking

CLEAN   = $(OBJECTS) $(TARGET)

$(TARGET): $(OBJECTS)
	$(CC) $(MY_LIBS) $(OBJECTS) -o $(TARGET) $(LDLIBS)

.cpp.o:
	$(CC) ${CFLAGS} -c $<

.cxx.o:
	$(CC) ${CFLAGS} -c $<

run: $(TARGET)
	$(TARGET)

clean:
	/bin/rm -f $(CLEAN)

