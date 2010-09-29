.SUFFIXES: .cxx
CC=g++
CFLAGS = -c -Wall
SOURCES = CarLikeModel.cxx Control.cxx Geometry.cxx MathFunctions.cxx Pioneer3ATState.cxx RobotModel.cxx RRT.cxx SkidSteerControlBased.cxx SkidSteerDynamicModel.cxx SkidSteerModel.cxx Utils.cxx World.cxx
OBJECTS = $(SOURCES:.cxx=.o)

all: $(SOURCES) $(OBJECTS)
    
.cxx.o:
	$(CC) $(CFLAGS) $< -o $@
