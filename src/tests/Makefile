# tests/Makefile
# Supports multiple standalone test files (each with its own main()),
# each buildable individually by name: `make control_test`, `make mapping_test`, etc.
# Shared rover source files (control.cpp, esp_comm.cpp, mapping.cpp) are pulled
# from the parent dir via VPATH -- main.cpp is never touched.

CXX = g++
CXXFLAGS = -std=c++17 -I.. -I../shared -I/usr/local/zed/include -I/usr/local/cuda-10.2/targets/aarch64-linux/include
LDFLAGS = -L/usr/local/zed/lib -lsl_zed \
          $(shell pkg-config --cflags --libs opencv4) \
          -lcuda -lcudart -L/usr/local/cuda-10.2/targets/aarch64-linux/lib

VPATH = ..

# ---- shared rover sources each test may depend on ----
# add more here as you split things out (e.g. mapping.cpp)
SHARED_SRC = control.cpp esp_comm.cpp
SHARED_OBJ = $(SHARED_SRC:.cpp=.o)

# ---- list every test here: name = the .cpp file without extension ----
# e.g. control_test.cpp -> control_test
TESTS = control_test

# building "make" with no args lists available tests instead of building everything
# (each test has its own main(), so they can't all be linked into one binary)
all:
	@echo "Available tests: $(TESTS)"
	@echo "Run 'make <test_name>' to build one, e.g. 'make control_test'"

# pattern rule: `make control_test` builds control_test.o + SHARED_OBJ -> ./control_test
$(TESTS): %: %.o $(SHARED_OBJ)
	$(CXX) $^ -o $@ $(LDFLAGS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f *.o $(TESTS)

.PHONY: all clean
