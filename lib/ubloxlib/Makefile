CXX = g++
INCLUDES = -Iinclude
CPPFLAGS = -Wall
BINDIR = build/bin
OBJDIR = build/obj

SOURCES = \
	src/protocol/packet.cpp \
	src/protocol/packetfinder.cpp \
	src/sensors/sensors.cpp \
	src/util/checksum.cpp \
	src/util/utilities.cpp \
	src/xplat/criticalsection.cpp \
	src/xplat/event.cpp \
	src/xplat/port.cpp \
	src/xplat/serialport.cpp \
	src/xplat/thread.cpp \
	src/xplat/ubtime.cpp
				
# Set the object file names, with the source directory stripped
# from the path, and the build path prepended in its place			
OBJECTS = $(SOURCES:src/%.cpp=$(OBJDIR)/%.o)

all: dirs lib

lib: libubcxx.a

libubcxx.a: $(OBJECTS)
	ar -cvq $(BINDIR)/libubcxx.a $(OBJECTS)

# Create the directories used in the build
.PHONY: dirs
dirs:
	@mkdir -p $(BINDIR)
	@mkdir -p $(OBJDIR)
	@mkdir -p $(dir $(OBJECTS))

$(OBJDIR)/%.o: src/%.cpp
	$(CXX) $(CPPFLAGS) $(INCLUDES) -c $< -o $@

clean:
	rm -f $(BINDIR)/libubcxx.a
	rm -f $(OBJECTS)
