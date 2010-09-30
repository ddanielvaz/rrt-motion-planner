.SUFFIXES: .cpp .cxx

CC = g++
INCLUDE_DIR = ../../include
CFLAGS  = -Wall -I$(INCLUDE_DIR) `pkg-config --cflags opencv`
LDLIBS  = -lm `pkg-config --libs opencv`

SRCS    = draw.cpp Carro.cxx Graphics.cxx Obstacles.cxx

OBJECTS = Carro.o Obstacles.o Graphics.o draw.o

PRE_COMPILED_OBJECTS = ../../lib/Geometry.o

TARGET = draw

CLEAN   = $(OBJECTS) $(TARGET)

$(TARGET): $(OBJECTS) $(PRE_COMPILED_OBJECTS)
	$(CC) $(PRE_COMPILED_OBJECTS) $(OBJECTS) -o $(TARGET) $(LDLIBS)

.cpp.o:
	$(CC) ${CFLAGS} -c $<

.cxx.o:
	$(CC) ${CFLAGS} -c $<

run: $(TARGET)
	$(TARGET)

clean:
	/bin/rm -f $(CLEAN)

