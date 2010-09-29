.SUFFIXES: .cxx
CC=g++
CFLAGS =-Wall -fPIC
SOURCES = CarLikeModel.cxx Control.cxx Geometry.cxx MathFunctions.cxx Pioneer3ATState.cxx RobotModel.cxx RRT.cxx SkidSteerControlBased.cxx SkidSteerDynamicModel.cxx SkidSteerModel.cxx Utils.cxx World.cxx
OBJECTS = $(SOURCES:.cxx=.o)

all: $(SOURCES) $(OBJECTS)
# 	@rm ../lib/libMotionPlanner.a ../lib/libMotionPlanner.so
# 	ar ruv ../lib/libMotionPlanner.a $(OBJECTS)
	$(CC) $(CFLAGS) -Wall -Wextra -nostartfiles -shared -Wl,-soname,libMotionPlanner.so.1 -o ../lib/libMotionPlanner.so $(OBJECTS) #-o ../lib/libMotionPlanner.so $(OBJECTS)

.cxx.o:
	$(CC) -shared $(CFLAGS) -c $< -o $@

clean: 
	@rm $(OBJECTS)
