.SUFFIXES: .cpp

CC = g++
# CFLAGS  = -Wall -O2
INCLUDE_DIR = ../include
CFLAGS  = -Wall -I$(INCLUDE_DIR)
LDLIBS  = -lm -lPQP -lemon

SRCS    = p3at.cpp

OBJECTS = p3at.o

TARGET = p3at.out

CLEAN   = $(OBJECTS) $(TARGET)

$(TARGET): $(OBJECTS)
	$(CC) $(OBJECTS) -o $(TARGET) $(LDLIBS)

.cpp.o:
	$(CC) ${CFLAGS} -c $<

run: $(TARGET)
	$(TARGET)

clean:
	/bin/rm -f $(CLEAN)

