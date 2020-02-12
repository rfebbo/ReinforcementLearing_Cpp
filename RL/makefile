INC_FILES := $(wildcard ./include/*.h)
SRC_FILES := $(wildcard ./src/*.cpp)
OBJ_FILES := $(patsubst ./src/%.cpp, ./obj/%.o, $(SRC_FILES))
TARGET := run
DEST = ./obj

CXX = g++

all: $(DEST) $(OBJ_FILES) $(TARGET)

$(DEST):
	mkdir -p $@
	echo $(OBJ_FILES)

%.o: ./src/%.cpp
	$(CXX) -o $(DEST)/$@ -c $< -I./include

$(TARGET): $(OBJ_FILES) $(INC_FILES) main.cpp
	$(CXX) -o $@ $(OBJ_FILES) main.cpp -I./include