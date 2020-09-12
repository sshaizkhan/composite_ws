ifndef SKINNING
SKINNING=SKINNING

ifndef CLEANFOLDER
CLEANFOLDER=SKINNING
endif

R ?= ../..
include $(R)/Makefile-headers/Makefile-header

# the object files to be compiled for this library
SKINNING_OBJECTS=skinning.o 

# the libraries this library depends on
SKINNING_LIBS=sparseMatrix quaternion matrixIO listIO basicAlgorithms

# the headers in this library
SKINNING_HEADERS=skinning.h

SKINNING_OBJECTS_FILENAMES=$(addprefix $(L)/skinning/, $(SKINNING_OBJECTS))
SKINNING_HEADER_FILENAMES=$(addprefix $(L)/skinning/, $(SKINNING_HEADERS))
SKINNING_LIB_MAKEFILES=$(call GET_LIB_MAKEFILES, $(SKINNING_LIBS))
SKINNING_LIB_FILENAMES=$(call GET_LIB_FILENAMES, $(SKINNING_LIBS))

include $(SKINNING_LIB_MAKEFILES)

all: $(L)/skinning/libskinning.a

$(L)/skinning/libskinning.a: $(SKINNING_OBJECTS_FILENAMES)
	ar r $@ $^; cp $@ $(L)/lib; cp $(L)/skinning/*.h $(L)/include

$(SKINNING_OBJECTS_FILENAMES): %.o: %.cpp $(SKINNING_LIB_FILENAMES) $(SKINNING_HEADER_FILENAMES)
	$(CXX) $(CXXFLAGS) -c $(INCLUDE) $< -o $@

ifeq ($(CLEANFOLDER), SKINNING)
clean: cleanskinning
endif

deepclean: cleanskinning

cleanskinning:
	$(RM) $(SKINNING_OBJECTS_FILENAMES) $(L)/skinning/libskinning.a

endif
