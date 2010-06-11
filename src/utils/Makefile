.SUFFIXES: .cpp

CC = g++
CFLAGS  = -Wall
LDLIBS  = -lm -lemon

SRCS    = estimate_torque.cpp

OBJECTS = estimate_torque.o

TARGET = torque

CLEAN   = $(OBJECTS) $(TARGET)

$(TARGET): $(OBJECTS)
	$(CC) $(OBJECTS) -o $(TARGET) $(LDLIBS)

.cpp.o:
	$(CC) ${CFLAGS} -c $<

run: $(TARGET)
	$(TARGET)

clean:
	/bin/rm -f $(CLEAN)

