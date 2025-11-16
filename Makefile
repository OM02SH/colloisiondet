CXX = clang++-18
CXXFLAGS = -std=c++23 -g -O2 \
  -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion \
  -Wfloat-equal -Wformat=2 -Wundef -Wpointer-arith -Wcast-qual \
  -Wcast-align -Wnon-virtual-dtor -Woverloaded-virtual -Wdouble-promotion

LDFLAGS = -ltinyxml2
app: ScenarioThreading.o
	$(CXX) $(CXXFLAGS) $^ $(LDFLAGS) -o $@

main.o: ScenarioThreading.cpp
	$(CXX) $(CXXFLAGS) -c $<