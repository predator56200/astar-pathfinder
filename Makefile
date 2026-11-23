CXX ?= g++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -O2

all: astar tests

astar: main.cpp pathfinder.hpp
	$(CXX) $(CXXFLAGS) main.cpp -o astar

tests: tests.cpp pathfinder.hpp
	$(CXX) $(CXXFLAGS) tests.cpp -o tests

test: tests
	./tests

demo: astar
	./astar maps/demo.txt --compare

clean:
	rm -f astar tests

.PHONY: all test demo clean
