# - Config file for the libnabo package
# It defines the following variables
#  libnabo_INCLUDE_DIRS - include directories for libnabo
#  libnabo_LIBRARIES    - libraries to link against
 
# Compute paths
get_filename_component(libnabo_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
set(libnabo_INCLUDE_DIRS "/home/aniruddha/Downloads/libnabo-master;/usr/local/include/eigen3")

if(CMAKE_COMPILER_IS_GNUCC)
	set(libnabo_LIBRARIES /usr/local/build/libnabo.a gomp)
else(CMAKE_COMPILER_IS_GNUCC)
	set(libnabo_LIBRARIES /usr/local/build/libnabo.a)
endif(CMAKE_COMPILER_IS_GNUCC)

# This causes catkin_simple to link against these libraries
set(libnabo_FOUND_CATKIN_PROJECT true)
