UNAME_S := ${shell uname -s}

# This option ensures we are using a relatively modern version of C++.
CXXFLAGS := -std=c++11 -g

# These are the locations to look for headers called from the .cpp files 
# Works only on linux and MacOS for now. TODO: Add windows support.
ifeq (${UNAME_S},Linux)
	INCLUDE := -Iheaders -I${MSKHOME}/mosek/8/tools/platform/linux64x86/h -Ithreadpool
endif
ifeq (${UNAME_S},Darwin)
	INCLUDE := -Iheaders -I${MSKHOME}/mosek/8/tools/platform/osx64x86/h -Ithreadpool
endif


# These are the locations and list of libraries to link to the binary.
# Works only for linux and MacOS for now. TODO: Add windows support.
ifeq (${UNAME_S},Linux)
	LDFLAGS := -L${MSKHOME}/mosek/8/tools/platform/linux64x86/bin \
                        -Wl,-rpath=${MSKHOME}/mosek/8/tools/platform/linux64x86/bin \
                        -pthread -lfusion64 -lmosek64
endif
ifeq (${UNAME_S},Darwin)
	LDFLAGS := -L${MSKHOME}/mosek/8/tools/platform/osx64x86/bin \
			-pthread -lfusion64 -lmosek64 
endif

# Define the location of dependencies folder, flags for CXX to output dependencies.
# See http://make.mad-scientist.net/papers/advanced-auto-dependency-generation/
DEPDIR := .deps
DEPFLAGS = -MT $@ -MMD -MP -MF $(DEPDIR)/$*.d

# Define source code to be every .cpp file in the src/ directory.
SRC := $(wildcard src/*.cpp) $(wildcard src/algorithms/*.cpp)

# Define object files to be the .o equivalent of every .cpp source file.  Similar for dependency files.
OBJ := $(SRC:%.cpp=build/%.o)
DEPFILES := $(SRC:%.cpp=$(DEPDIR)/%.d)

# The first, and default, target is the program which depends on object files and the threadpool.
prog: $(OBJ) threadpool/libthpool.a
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

# Object files depend on the .cpp file and .d file, and the .dep directory which should exist first.
build/%.o: %.cpp $(DEPDIR)/%.d | $(DEPDIR)
	@mkdir -p $(@D)
	@mkdir -p $(word 2, $(^D))
	$(CXX) $(CXXFLAGS) $(DEPFLAGS) $(INCLUDE) -c $< -o $@

threadpool/libthpool.a:
	cc -c threadpool/thpool.c -o threadpool/thpool.o
	ar rc threadpool/libthpool.a threadpool/thpool.o

$(DEPDIR):
	@mkdir -p $@

# Tell make not to panic if a .d file doesn't exist yet.
$(DEPFILES):

# Import dependency files, that exist, to recompile objects if headers change. (make uses two passes)
include $(wildcard $(DEPFILES))

.PHONY: clean

clean:
	rm -f prog $(OBJ) $(DEPFILES) threadpool/libthpool.a
