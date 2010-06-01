.SUFFIXES: .cpp

CC = g++
CFLAGS  = -Wall -O2
LDLIBS  = -lm -lPQP -lemon

SRCS    = main.cpp

OBJECTS = main.o

TARGET = planning

CLEAN   = $(OBJECTS) $(TARGET)

$(TARGET): $(OBJECTS)
	$(CC) $(OBJECTS) -o $(TARGET) $(LDLIBS)

.cpp.o:
	$(CC) ${CFLAGS} -c $<

run: $(TARGET)
	$(TARGET)

clean:
	/bin/rm -f $(CLEAN)

