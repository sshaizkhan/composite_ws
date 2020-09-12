ifndef CONSTRAINTS
CONSTRAINTS=CONSTRAINTS

ifndef CLEANFOLDER
CLEANFOLDER=CONSTRAINTS
endif

R ?= ../..
include $(R)/Makefile-headers/Makefile-header

# the object files to be compiled for this library
CONSTRAINTS_OBJECTS=constraints.o

# the libraries this library depends on
CONSTRAINTS_LIBS=volumetricMesh sparseMatrix skinning mesh

# the headers in this library
CONSTRAINTS_HEADERS=constraints.h

CONSTRAINTS_OBJECTS_FILENAMES=$(addprefix $(L)/constraints/, $(CONSTRAINTS_OBJECTS))
CONSTRAINTS_HEADER_FILENAMES=$(addprefix $(L)/constraints/, $(CONSTRAINTS_HEADERS))
CONSTRAINTS_LIB_MAKEFILES=$(call GET_LIB_MAKEFILES, $(CONSTRAINTS_LIBS))
CONSTRAINTS_LIB_FILENAMES=$(call GET_LIB_FILENAMES, $(CONSTRAINTS_LIBS))

include $(CONSTRAINTS_LIB_MAKEFILES)

all: $(L)/constraints/libconstraints.a

$(L)/constraints/libconstraints.a: $(CONSTRAINTS_OBJECTS_FILENAMES)
	ar r $@ $^; cp $@ $(L)/lib; cp $(L)/constraints/*.h $(L)/include

$(CONSTRAINTS_OBJECTS_FILENAMES): %.o: %.cpp $(CONSTRAINTS_LIB_FILENAMES) $(CONSTRAINTS_HEADER_FILENAMES)
	$(CXX) $(CXXFLAGS) -c $(INCLUDE) $< -o $@

ifeq ($(CLEANFOLDER), CONSTRAINTS)
clean: cleanconstraints
endif

deepclean: cleanconstraints

cleanconstraints:
	$(RM) $(CONSTRAINTS_OBJECTS_FILENAMES) $(L)/constraints/libconstraints.a

endif
