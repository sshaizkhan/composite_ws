ifndef VIRTUALTETS
VIRTUALTETS=VIRTUALTETS

ifndef CLEANFOLDER
CLEANFOLDER=VIRTUALTETS
endif

R ?= ../../..
include $(R)/Makefile-headers/Makefile-header

# the object files to be compiled for this library
VIRTUALTETS_OBJECTS=virtualTets.o geometryQueryER.o  tetTriPiece.o tetTriMeshCutting.o tetTriCutFeature.o

# the libraries this library depends on
VIRTUALTETS_LIBS=interpolationCoordinates LGPL/exactArithmetic mesh basicAlgorithms

# the headers in this library
VIRTUALTETS_HEADERS=virtualTets.h geometryQueryER.h  tetTriPiece.h tetTriCuttingData.h tetTriMeshCutting.h tetTriCutFeature.h

VIRTUALTETS_OBJECTS_FILENAMES=$(addprefix $(L)/LGPL/virtualTets/, $(VIRTUALTETS_OBJECTS))
VIRTUALTETS_HEADER_FILENAMES=$(addprefix $(L)/LGPL/virtualTets/, $(VIRTUALTETS_HEADERS))
VIRTUALTETS_LIB_MAKEFILES=$(call GET_LIB_MAKEFILES, $(VIRTUALTETS_LIBS))
VIRTUALTETS_LIB_FILENAMES=$(call GET_LIB_FILENAMES, $(VIRTUALTETS_LIBS))

include $(VIRTUALTETS_LIB_MAKEFILES)

all: $(L)/LGPL/virtualTets/libvirtualTets.a

$(L)/LGPL/virtualTets/libvirtualTets.a: $(VIRTUALTETS_OBJECTS_FILENAMES)
	ar r $@ $^; cp $@ $(L)/lib; cp $(L)/LGPL/virtualTets/*.h $(L)/include

$(VIRTUALTETS_OBJECTS_FILENAMES): %.o: %.cpp $(VIRTUALTETS_LIB_FILENAMES) $(VIRTUALTETS_HEADER_FILENAMES)
	$(CXX) $(CXXFLAGS) -c $(INCLUDE) $< -o $@

#$(L)/LGPL/virtualTets/virtualTetsTest: $(L)/LGPL/virtualTets/virtualTetsTest.o $(L)/LGPL/virtualTets/libvirtualTets.a
#	$(CXXLD) $(LDFLAGS) $^ $(LIBRARYPATH) $(addprefix -l, $(VIRTUALTETS_LIBS)) $(STANDARD_LIBS) -o $@

ifeq ($(CLEANFOLDER), VIRTUALTETS)
clean: cleanvirtualTets
endif

deepclean: cleanvirtualTets

cleanvirtualTets:
	$(RM) $(VIRTUALTETS_OBJECTS_FILENAMES) $(L)/LGPL/virtualTets/libvirtualTets.a

endif
	
