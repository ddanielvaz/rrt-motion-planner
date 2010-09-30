.SUFFIXES: .cxx
CC=g++
INCLUDE_DIR = ./include/
CFLAGS = -c -Wall -I$(INCLUDE_DIR)
SOURCES = CarLikeModel.cxx Control.cxx Geometry.cxx MathFunctions.cxx Pioneer3ATState.cxx RobotModel.cxx RRT.cxx SkidSteerControlBased.cxx SkidSteerDynamicModel.cxx SkidSteerModel.cxx Utils.cxx World.cxx
OBJECTS = $(SOURCES:.cxx=.o)
MY_LIB = ./lib/libMotionPlanner.a

all: $(SOURCES) $(OBJECTS)
	ar ruv $(MY_LIB) $(OBJECTS)
	@cp $(OBJECTS) ./lib/

.cxx.o:
	$(CC) $(CFLAGS) $< -o $@

clean: 
	@rm $(OBJECTS) $(MY_LIB)
