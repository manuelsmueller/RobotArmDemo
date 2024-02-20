PROJECT_ROOT = $(dir $(abspath $(lastword $(MAKEFILE_LIST))))

CXXARGS = -g -std=c++11 -D _DEBUG
LDFLAGS = -lpq -lyaml -pthread

all : visualization

visualization : 
	g++ $(CXXARGS) $(PROJECT_ROOT)*.cpp $(PROJECT_ROOT)/src/*.cpp \
	-o visualization -lGL -lSDL2 -lGLEW $(LDFLAGS)
clean : 
	rm visualization
