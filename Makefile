CPP = clang++
CPPFLAGS = -std=c++17 -Wall -Werror -I/home/leaf/.local/lib/or-tools/include
LINKFLAGS = -L/home/leaf/.local/lib/or-tools/lib -lortools -Wl,-rpath,/home/leaf/.local/lib/or-tools/lib
SRC = $(wildcard src/*.cpp) $(wildcard src/*/*.cpp)
OBJ = $(SRC:.cpp=.o)
BIN = ll

%.o: %.cpp
	$(CPP) $(CPPFLAGS) -g -c $< -o $@

$(BIN): $(OBJ)
	$(CPP) $(OBJ) -o $(BIN) $(LINKFLAGS)

.PHONY: clean
clean:
	@rm $(OBJ)
	@rm $(BIN)
