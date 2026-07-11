# Compiler and flags
CXX = g++
CXXFLAGS = -I/usr/local/zed/include -I/usr/local/cuda-10.2/targets/aarch64-linux/include
LDFLAGS = -L/usr/local/zed/lib -lsl_zed \
          $(shell pkg-config --cflags --libs opencv4) \
          -lcuda -lcudart -L/usr/local/cuda-10.2/targets/aarch64-linux/lib

# Automatically find all .cpp files in this folder
SOURCES = $(wildcard *.cpp)

# Turn each file.cpp into file.o
OBJECTS = $(SOURCES:.cpp=.o)

# Name of the final program
TARGET = rover

# Default target - just running "make" builds this
all: $(TARGET)

# Link all the .o files together into the final executable
$(TARGET): $(OBJECTS)
	$(CXX) $(OBJECTS) -o $(TARGET) $(LDFLAGS)

# Compile each .cpp into a .o, only if that .cpp changed
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# "make clean" deletes all built files, forcing a fresh rebuild next time
clean:
	rm -f $(OBJECTS) $(TARGET)
