CC		  := g++

#The Target Binary Program
TARGET := main

TEST := tests

#The Directories, Source, Includes, Objects, Binary and Resources
SRCDIR    := src
INCDIR    := src
BUILDDIR  := build
TARGETDIR := bin
RESDIR    := res
SRCEXT    := cpp
DEPEXT    := d
OBJEXT    := o

#Flags, Libraries and Includes
CFLAGS := -Wall -Wextra -std=c++17 -ggdb
#LIBVN := lib/vnproglib-1.1.5.0
#LIBUB := lib/ubloxlib
LIB := -lgtest \
		-lprofiler \
		-lpthread \
		-ldocopt #\
		-L$(LIBVN)/build/bin -lvncxx \
		-L$(LIBUB)/build/bin -lubcxx 
INC := -I$(INCDIR) #\
		-isystem $(LIBVN)/include \
		-isystem $(LIBUB)/include
INCDEP := -I$(INCDIR) #\
			-isystem $(LIBVN)/include \
			-isystem $(LIBUB)/include

#---------------------------------------------------------------------------------
#DO NOT EDIT BELOW THIS LINE
#---------------------------------------------------------------------------------
SOURCES := $(shell find $(SRCDIR) -type f -name *.$(SRCEXT))
OBJECTSALL := $(patsubst $(SRCDIR)/%,$(BUILDDIR)/%,$(SOURCES:.$(SRCEXT)=.$(OBJEXT)))
OBJECTS := $(patsubst build/$(TEST).o,,$(OBJECTSALL))
OBJECTSTEST := $(patsubst build/$(TARGET).o,,$(OBJECTSALL))

#Default Make$(LIBVN)/build/bin/libvncxx.a $(LIBUB)/build/bin/libubcxx.a
all: resources  $(TARGET)

run: all
	./$(TARGETDIR)/$(TARGET)

remake: clean all

remake-run: clean run

test: resources $(LIBVN)/build/bin/libvncxx.a $(LIBUB)/build/bin/libubcxx.a $(TEST)

test-run: test
	-./$(TARGETDIR)/$(TEST) --gtest_output=xml:tests/tests.xml
	xsltproc tests/gtest2html.xslt tests/tests.xml > tests/tests.html
	-rm tests/*.xml
	xdg-open tests/tests.html

test-remake: clean test

test-remake-run: clean test-run

profiler: remake
	-CPUPROFILE=tests/prof.out ./$(TARGETDIR)/$(TARGET)
	-pprof --gv $(TARGETDIR)/$(TARGET) tests/prof.out
	# No nodes to print:
	# Your program might be running too quickly to produce any profiling samples.
	# You will need at least few seconds of samples to have near-meaningful profile.
	# You can also look at cpu profiling frequency.

test-profiler: test-remake
	-CPUPROFILE=tests/profTests.out ./$(TARGETDIR)/$(TEST)
	-pprof --gv $(TARGETDIR)/$(TEST) tests/profTests.out
	# No nodes to print:
	# Your program might be running too quickly to produce any profiling samples.
	# You will need at least few seconds of samples to have near-meaningful profile.
	# You can also look at cpu profiling frequency.

valgrind-profiler: remake
	valgrind --callgrind-out-file=tests/callgrind.out --tool=callgrind $(TARGETDIR)/$(TARGET)
	kcachegrind tests/callgrind.out

test-valgrind-profiler: test-remake
	valgrind --callgrind-out-file=tests/callgrindTests.out --tool=callgrind $(TARGETDIR)/$(TEST)
	kcachegrind tests/callgrindTests.out

valgrind-leakcheck: remake
	valgrind --leak-check=yes $(TARGETDIR)/$(TARGET)

test-valgrind-leakcheck: test-remake
	valgrind --leak-check=yes $(TARGETDIR)/$(TEST)

documentation:
	doxygen doc/config

#Copy Resources from Resources Directory to Target Directory
resources: directories
#@cp $(RESDIR)/* $(TARGETDIR)/

#Make the Directories
directories:
	@mkdir -p $(TARGETDIR)
	@mkdir -p $(BUILDDIR)

#Clean only Objects
clean:
	@$(RM) -rf $(BUILDDIR)

#Full Clean, Objects and Binaries
cleaner: clean
	@$(RM) -rf $(TARGETDIR)
	-find . -name "*.orig" -type f -delete
	cd $(LIBVN) && make clean
	cd $(LIBUB) && make clean

#Pull in dependency info for *existing* .o files
-include $(OBJECTS:.$(OBJEXT)=.$(DEPEXT))

#Link
$(LIBVN)/build/bin/libvncxx.a:
	cd $(LIBVN) && make
#Link
$(LIBUB)/build/bin/libubcxx.a:
	cd $(LIBUB) && make

#Link
$(TARGET): $(OBJECTS)
	$(CC) -o $(TARGETDIR)/$(TARGET) $^ $(LIB)
	
#Link
$(TEST): $(OBJECTSTEST)
	$(CC) -o $(TARGETDIR)/$(TEST) $^ $(LIB)

#Compile
$(BUILDDIR)/%.$(OBJEXT): $(SRCDIR)/%.$(SRCEXT)
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(INC) -c -o $@ $<
	@$(CC) $(CFLAGS) $(INCDEP) -MM $(SRCDIR)/$*.$(SRCEXT) > $(BUILDDIR)/$*.$(DEPEXT)
	@cp -f $(BUILDDIR)/$*.$(DEPEXT) $(BUILDDIR)/$*.$(DEPEXT).tmp
	@sed -e 's|.*:|$(BUILDDIR)/$*.$(OBJEXT):|' < $(BUILDDIR)/$*.$(DEPEXT).tmp > $(BUILDDIR)/$*.$(DEPEXT)
	@sed -e 's/.*://' -e 's/\\$$//' < $(BUILDDIR)/$*.$(DEPEXT).tmp | fmt -1 | sed -e 's/^ *//' -e 's/$$/:/' >> $(BUILDDIR)/$*.$(DEPEXT)
	@rm -f $(BUILDDIR)/$*.$(DEPEXT).tmp

#Non-File Targets
.PHONY: all remake clean cleaner resources