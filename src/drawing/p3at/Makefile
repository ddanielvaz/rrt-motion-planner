.SUFFIXES: .cpp

CC = g++
INCLUDE_DIR = ../../include
CFLAGS  = -Wall -I$(INCLUDE_DIR) `pkg-config --cflags opencv`
LDLIBS  = -lm `pkg-config --libs opencv`

SRCS    = draw.cpp

OBJECTS = draw.o

TARGET = draw

CLEAN   = $(OBJECTS) $(TARGET)

$(TARGET): $(OBJECTS)
	$(CC) $(OBJECTS) -o $(TARGET) $(LDLIBS)

.cpp.o:
	$(CC) ${CFLAGS} -c $<

run: $(TARGET)
	$(TARGET)

clean:
	/bin/rm -f $(CLEAN)

