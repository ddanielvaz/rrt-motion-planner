.SUFFIXES: .cxx
CC=g++
INCLUDE_DIR = ./include/
CFLAGS = -c -Wall -I$(INCLUDE_DIR)
SOURCES = CarLikeModel.cxx Control.cxx Geometry.cxx MathFunctions.cxx Pioneer3ATState.cxx RobotModel.cxx RRT.cxx SkidSteerControlBased.cxx SkidSteerDynamicModel.cxx SkidSteerModel.cxx Utils.cxx World.cxx
OBJDIR = ./lib
MY_LIB = ./lib/libMotionPlanner.a
OBJECTS = $(addprefix $(OBJDIR)/,$(SOURCES:.cxx=.o))

all: $(SOURCES) $(OBJECTS)
	@ar ruv $(MY_LIB) $(OBJECTS)

$(OBJDIR)/%.o: %.cxx
	$(CC) $(CFLAGS) $< -o $@

clean:
	@rm $(OBJECTS)

dist-clean:
	@rm $(MY_LIB)
