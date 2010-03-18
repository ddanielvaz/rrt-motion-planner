.SUFFIXES: .cpp

CC = g++
CFLAGS  = -Wall -O2 `pkg-config --cflags opencv`
LDLIBS  = -lm -lPQP `pkg-config --libs opencv`

SRCS    = main.cpp

OBJECTS = main.o

TARGET = planning

CLEAN   = $(OBJECTS) $(TARGET)

$(TARGET): $(OBJECTS)
	$(CC) $(CFLAGS) -o $(TARGET) $(OBJECTS) $(LDLIBS)

.cpp.o:
	$(CC) ${CFLAGS} -c $<

run: $(TARGET)
	$(TARGET)

clean:
	/bin/rm -f $(CLEAN)

