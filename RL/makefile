INC_FILES := $(wildcard ./include/*.h)
SRC_FILES := $(wildcard ./src/*.cpp)
OBJ_FILES := $(patsubst ./src/%.cpp, ./obj/%.o, $(SRC_FILES))
TARGET := run
DEST = ./obj

CXX = g++

all: $(INC_FILES) $(DEST) $(OBJ_FILES) $(TARGET)

clean:
	rm -rf $(DEST) $(TARGET)

$(DEST):
	mkdir -p $@
	echo $(OBJ_FILES)

$(OBJ_FILES): $(INC_FILES)

obj/%.o: ./src/%.cpp
	$(CXX) -o $@ -c $< -I./include

$(TARGET): $(OBJ_FILES) main.cpp
	$(CXX) -o $@ $(OBJ_FILES) main.cpp -I./include