.SUFFIXES: .cxx
CC=g++
INCLUDE_DIR = ./include/
CFLAGS = -c -Wall -I$(INCLUDE_DIR)
SOURCES = CarLikeModel.cxx TrackingControl.cxx CaraccioloControl.cxx FierroControl.cxx Geometry.cxx MathFunctions.cxx Pioneer3ATState.cxx RobotModel.cxx RRT.cxx SkidSteerControlBased.cxx SkidSteerDynamicModel.cxx SkidSteerModel.cxx World.cxx StateSampler.cxx DistanceMeter.cxx
OBJDIR = ./lib
MY_LIB = ./lib/libMotionPlanner.a
OBJECTS = $(addprefix $(OBJDIR)/,$(SOURCES:.cxx=.o))
CLEAN = $(OBJECTS) $(MY_LIB)

all: $(SOURCES) $(OBJECTS)
	@ar ruv $(MY_LIB) $(OBJECTS)
	$(MAKE) -C tests

$(OBJDIR)/%.o: %.cxx
	$(CC) $(CFLAGS) $< -o $@

clean:
	@rm $(CLEAN)
	$(MAKE) -C tests clean

clean-lib:
	@rm $(MY_LIB)

clean-objects:
	@rm $(OBJECTS)

clean-tests:
	$(MAKE) -C tests clean
