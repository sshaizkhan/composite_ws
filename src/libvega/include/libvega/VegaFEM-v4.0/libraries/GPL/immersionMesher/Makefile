ifndef IMMERSIONMESHER
IMMERSIONMESHER=IMMERSIONMESHER

ifndef CLEANFOLDER
CLEANFOLDER=IMMERSIONMESHER
endif

R ?= ../../..
include $(R)/Makefile-headers/Makefile-header

# the object files to be compiled for this library
IMMERSIONMESHER_OBJECTS=immersionMesherGenerateTetMesh.o immersionMesherCellComplex.o immersionGraphNode.o immersionMesherBuildGraph.o virtualTets-via-csg.o

# the libraries this library depends on
IMMERSIONMESHER_LIBS=LGPL/virtualTets LGPL/exactArithmetic mesh basicAlgorithms performanceCounter GPL/libiglInterface

# the headers in this library
IMMERSIONMESHER_HEADERS=immersionMesher.h immersionGraphNode.h virtualTets-via-csg.h

IMMERSIONMESHER_OBJECTS_FILENAMES=$(addprefix $(L)/GPL/immersionMesher/, $(IMMERSIONMESHER_OBJECTS))
IMMERSIONMESHER_HEADER_FILENAMES=$(addprefix $(L)/GPL/immersionMesher/, $(IMMERSIONMESHER_HEADERS))
IMMERSIONMESHER_LIB_MAKEFILES=$(call GET_LIB_MAKEFILES, $(IMMERSIONMESHER_LIBS))
IMMERSIONMESHER_LIB_FILENAMES=$(call GET_LIB_FILENAMES, $(IMMERSIONMESHER_LIBS))

include $(IMMERSIONMESHER_LIB_MAKEFILES)

all: $(L)/GPL/immersionMesher/libimmersionMesher.a

$(L)/GPL/immersionMesher/libimmersionMesher.a: $(IMMERSIONMESHER_OBJECTS_FILENAMES)
	ar r $@ $^; cp $@ $(L)/lib; cp $(L)/GPL/immersionMesher/*.h $(L)/include

$(IMMERSIONMESHER_OBJECTS_FILENAMES): %.o: %.cpp $(IMMERSIONMESHER_LIB_FILENAMES) $(IMMERSIONMESHER_HEADER_FILENAMES)
	$(CXX) $(CXXFLAGS) -c $(INCLUDE) $< -o $@

#$(L)/GPL/immersionMesher/immersionMesherTest: $(L)/GPL/immersionMesher/immersionMesherTest.o $(L)/GPL/immersionMesher/libimmersionMesher.a
#	$(CXXLD) $(LDFLAGS) $^ $(LIBRARYPATH) $(addprefix -l, $(IMMERSIONMESHER_LIBS)) $(STANDARD_LIBS) -o $@

ifeq ($(CLEANFOLDER), IMMERSIONMESHER)
clean: cleanimmersionMesher
endif

deepclean: cleanimmersionMesher

cleanimmersionMesher:
	$(RM) $(IMMERSIONMESHER_OBJECTS_FILENAMES) $(L)/GPL/immersionMesher/libimmersionMesher.a

endif
	
