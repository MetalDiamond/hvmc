.SUFFIXES:
.SUFFIXES: .cpp .o

CXX=g++
CXXFLAGS=-Wall -Werror -g -std=c++11

SDL_PATH=deps/SDL2-2.0.3
SDL_INC=-I$(SDL_PATH)/include
SDL_LIB=-L$(SDL_PATH)/lib -lSDL2
INCS=${SDL_INC}
LIBS=${SDL_LIB}

OBJDIR=build
SRCDIR=src
HDRDIR=src

SOURCES=$(shell find $(SRCDIR) -name '*.cpp')
HEADERS=$(shell find $(HDRDIR) -name '*.h'  )
OBJECTS=$(patsubst $(SRCDIR)/%.cpp,$(OBJDIR)/%.o,$(SOURCES))
EXECUTABLE=hvmc

all: $(EXECUTABLE) $(SOURCES) $(HEADERS)

$(EXECUTABLE): buildrepo $(OBJECTS)
	$(CXX) $(OBJECTS) -o $@ $(LIBS)

$(OBJDIR)/%.o: $(SRCDIR)/%.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@ $(INCS)

clean:
	rm $(OBJECTS)

mrproper: clean
	rm hvmc

buildrepo:
	@$(call make-repo)

define make-repo
	mkdir -p $(OBJDIR)
endef
