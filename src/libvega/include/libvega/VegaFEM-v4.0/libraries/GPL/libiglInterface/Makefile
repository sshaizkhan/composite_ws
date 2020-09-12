ifndef LIBIGLINTERFACE
LIBIGLINTERFACE=LIBIGLINTERFACE

ifndef CLEANFOLDER
CLEANFOLDER=LIBIGLINTERFACE
endif

R ?= ../../..
include $(R)/Makefile-headers/Makefile-header

# the object files to be compiled for this library
LIBIGLINTERFACE_OBJECTS=iglRemeshSelfIntersection.o 

# the libraries this library depends on
LIBIGLINTERFACE_LIBS=mesh LGPL/exactArithmetic

# the headers in this library
LIBIGLINTERFACE_HEADERS=iglRemeshSelfIntersection.h

LIBIGLINTERFACE_OBJECTS_FILENAMES=$(addprefix $(L)/GPL/libiglInterface/, $(LIBIGLINTERFACE_OBJECTS))
LIBIGLINTERFACE_HEADER_FILENAMES=$(addprefix $(L)/GPL/libiglInterface/, $(LIBIGLINTERFACE_HEADERS))
LIBIGLINTERFACE_LIB_MAKEFILES=$(call GET_LIB_MAKEFILES, $(LIBIGLINTERFACE_LIBS))
LIBIGLINTERFACE_LIB_FILENAMES=$(call GET_LIB_FILENAMES, $(LIBIGLINTERFACE_LIBS))

include $(LIBIGLINTERFACE_LIB_MAKEFILES)

all: $(L)/GPL/libiglInterface/liblibiglInterface.a

$(L)/GPL/libiglInterface/liblibiglInterface.a: $(LIBIGLINTERFACE_OBJECTS_FILENAMES)
	ar r $@ $^; cp $@ $(L)/lib; cp $(L)/GPL/libiglInterface/*.h $(L)/include

$(LIBIGLINTERFACE_OBJECTS_FILENAMES): %.o: %.cpp $(LIBIGLINTERFACE_LIB_FILENAMES) $(LIBIGLINTERFACE_HEADER_FILENAMES)
	$(CXX) $(CXXFLAGS) -c $(INCLUDE) $(EIGEN_INCLUDE) $(CGAL_INCLUDE) $(IGL_INCLUDE) $< -o $@

#$(L)/GPL/libiglInterface/libiglInterfaceTest: $(L)/GPL/libiglInterface/libiglInterfaceTest.o $(L)/GPL/libiglInterface/liblibiglInterface.a
#	$(CXXLD) $(LDFLAGS) $^ $(LIBRARYPATH) $(addprefix -l, $(LIBIGLINTERFACE_LIBS)) $(STANDARD_LIBS) -o $@

ifeq ($(CLEANFOLDER), LIBIGLINTERFACE)
clean: cleanlibiglInterface
endif

deepclean: cleanlibiglInterface

cleanlibiglInterface:
	$(RM) $(LIBIGLINTERFACE_OBJECTS_FILENAMES) $(L)/GPL/libiglInterface/liblibiglInterface.a

endif
	
