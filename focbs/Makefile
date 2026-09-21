CXX ?= g++
CPPFLAGS ?= -Iinclude
CXXFLAGS ?= -std=c++17 -O2 -DNDEBUG -DFOCBS_REVIEW_CLI

TARGET := focbs
SOURCES := \
	app/main.cpp \
	src/cbs.cpp \
	src/cbs_trace.cpp \
	src/common.cpp \
	src/conflict.cpp \
	src/conflict_detection.cpp \
	src/constraint_table.cpp \
	src/idle_oracle.cpp \
	src/instance.cpp \
	src/lex_astar.cpp \
	src/stp_astar.cpp

# Link Boost, libstdc++, and libgcc statically; glibc and pthread stay dynamic.
LDLIBS := -Wl,-Bstatic -lboost_program_options -Wl,-Bdynamic -pthread \
	-static-libstdc++ -static-libgcc

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(SOURCES) $(wildcard include/*.h include/*.hpp)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(SOURCES) $(LDLIBS) -o $@

clean:
	rm -f $(TARGET)
