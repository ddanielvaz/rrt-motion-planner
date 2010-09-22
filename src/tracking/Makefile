.SUFFIXES: .cpp

CC = g++
INCLUDE_DIR = ../include
CFLAGS  = -Wall -I$(INCLUDE_DIR) `pkg-config --cflags opencv playerc++`
LDLIBS  = -lm -lemon `pkg-config --libs playerc++`

SRCS    = tracking.cpp

OBJECTS = tracking.o

TARGET = tracking

CLEAN   = $(OBJECTS) $(TARGET)

$(TARGET): $(OBJECTS)
	$(CC) $(OBJECTS) -o $(TARGET) $(LDLIBS)

.cpp.o:
	$(CC) ${CFLAGS} -c $<

run: $(TARGET)
	$(TARGET)

clean:
	/bin/rm -f $(CLEAN)

