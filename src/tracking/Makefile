.SUFFIXES: .cpp .cxx

CC = g++
INCLUDE_DIR = ../include
CFLAGS  = -Wall -I$(INCLUDE_DIR) `pkg-config --cflags opencv playerc++`
LDLIBS  = -lm -lemon `pkg-config --libs playerc++`

SRCS    = tracking.cpp PlayerTracking.cxx RobotProxy.cxx Position2DProxy.cxx

OBJECTS = Position2DProxy.o RobotProxy.o PlayerTracking.o tracking.o

MY_LIBS = ../lib/libMotionPlanner.a

TARGET = tracking.out

CLEAN   = $(OBJECTS) $(TARGET)

$(TARGET): $(OBJECTS) $(MY_LIBS)
	$(CC) $(OBJECTS) $(MY_LIBS) -o $(TARGET) $(LDLIBS)
	@cp $(TARGET) ../tests/

.cpp.o:
	$(CC) ${CFLAGS} -c $<

.cxx.o:
	$(CC) ${CFLAGS} -c $<

run: $(TARGET)
	$(TARGET)

clean:
	/bin/rm -f $(CLEAN)
	/bin/rm ../tests/tracking.out

