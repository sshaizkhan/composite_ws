ifndef CLOTHBWRT
CLOTHBWRT=CLOTHBWRT

ifndef CLEANFOLDER
CLEANFOLDER=CLOTHBWRT
endif

#fcl
# FCL_DIR=/home/cam_sanding/fcl
# FCL_DIR=/usr/local
# FCL_INCLUDE=-I$(FCL_DIR)/include/fcl
# FCL_LIB=-lfcl -L$(FCL_DIR)/lib

# SC_DIR=/home/cam_sanding/sheet_collision
# SC_INCLUDE=-I$(SC_DIR)/include/
# SC_LIB=-L$(SC_DIR)/src

R ?= ../..
include $(R)/Makefile-headers/Makefile-header

# the object files to be compiled for this utility
CLOTHBWRT_OBJECTS=cloth-simple.o
#clothBW-rt.o initGraphics.o

# the libraries this utility depends on
CLOTHBWRT_LIBS=stencilForceModel objMesh imageIO sceneObject objMesh imageIO clothBW integratorSparse integrator sparseSolver forceModel sparseMatrix lighting configFile constraints volumetricMesh mesh skinning quaternion listIO getopts camera minivector openGLHelper basicAlgorithms constrainedDOFs matrixIO

# the headers in this library
CLOTHBWRT_HEADERS=#initGraphics.h

CLOTHBWRT_LINK=$(addprefix -l, $(CLOTHBWRT_LIBS)) $(SPOOLES_LIB) $(BLASLAPACK_LIB) $(GLUI_LIB) $(CG_LIBS) $(IMAGE_LIBS) $(TBB_LIB) $(STANDARD_LIBS)

CLOTHBWRT_OBJECTS_FILENAMES=$(addprefix $(R)/utilities/clothBW-rt/, $(CLOTHBWRT_OBJECTS))
CLOTHBWRT_HEADER_FILENAMES=$(addprefix $(R)/utilities/clothBW-rt/, $(CLOTHBWRT_HEADERS))
CLOTHBWRT_LIB_MAKEFILES=$(call GET_LIB_MAKEFILES, $(CLOTHBWRT_LIBS))
CLOTHBWRT_LIB_FILENAMES=$(call GET_LIB_FILENAMES, $(CLOTHBWRT_LIBS))

SC_DIR=/home/cam_sanding/sheet_collision
SC_OBJ=SheetCollision.cpp.o ReadFile.cpp.o
# SC_OBJ_FILENAMES = $(SC_DIR)/build/CMakeFiles/fclMain.dir/src/$(SC_OBJ)
SC_OBJ_FILENAMES = $(addprefix $(SC_DIR)/build/CMakeFiles/fclMain.dir/src/, $(SC_OBJ))
SC_HPP=SheetCollision.hpp ReadFile.hpp
SC_HPP_FILENAMES = $(addprefix $(SC_DIR)/include/, $(SC_HPP))
SC_SRC=$(SC_DIR)/src/SheetCollision.cpp $(SC_DIR)/src/ReadFile.cpp
SC_INCLUDE=-I$(SC_DIR)/include 
IGL_LIB=-L$(SC_DIR)/libigl -ligl
FCL_LIB=-L/usr/local/lib/ -lfcl -lccd
# SC_LINK=-L/home/cam_sanding/sheet_collision/libigl/libigl.a -rdynamic /usr/local/lib/libfcl.so /usr/local/lib/libccd.so.2.0 ../libigl/libigl.a -lm -Wl,-rpath,/usr/local/lib:/home/cam_sanding/sheet_collision/libigl/libigl.a 
SC_LINK=$(FCL_LIB) $(IGL_LIB)
include $(CLOTHBWRT_LIB_MAKEFILES) ../../libraries/glui/Makefile 

all: $(R)/utilities/clothBW-rt/clothBW-simple

CURRENT_DIR = $(shell pwd)
$(R)/utilities/clothBW-rt/clothBW-simple: $(CLOTHBWRT_OBJECTS_FILENAMES) 
	$(CXXLD) $(LDFLAGS) $(CLOTHBWRT_OBJECTS) $(SC_OBJ_FILENAMES) $(CLOTHBWRT_LINK) $(SC_LINK) -Wl,-rpath,$(CURRENT_DIR)/$(GLUI_DIR)/lib -o $@; cp $@ $(R)/utilities/bin/

$(CLOTHBWRT_OBJECTS_FILENAMES):  %.o: %.cpp $(CLOTHBWRT_LIB_FILENAMES) $(CLOTHBWRT_HEADER_FILENAMES) 
	$(CXX) $(CXXFLAGS) -c $(TBB_INCLUDE) $(INCLUDE) $(CG_INCLUDE) $(GLUI_INCLUDE) $(SC_INCLUDE) $< -o $@

$(SC_OBJ_FILENAMES): $(SC_SRC) $(SC_HPP_FILENAMES) $(SC_LINK)
	$(CXX) $(CXXFLAGS) -c $(INCLUDE) $< -o $@


ifeq ($(CLEANFOLDER), CLOTHBWRT)
clean: cleanclothBW-rt
endif

deepclean: cleanclothBW-rt

cleanclothBW-rt:
	$(RM) $(CLOTHBWRT_OBJECTS_FILENAMES) $(R)/utilities/clothBW-rt/clothBW-rt

endif

