cmake_minimum_required(VERSION 2.8)

project( examples )

find_package( OpenCV REQUIRED )
find_package( OpenGL )

if(OPENGL_FOUND)
  include_directories( ${OpenCV_INCLUDE_DIRS} ${OPENGL_INCLUDE_DIRS} )
else()
  include_directories( ${OpenCV_INCLUDE_DIRS} )
endif()

#########################
# FOR LEARNING OPENCV 3 #
#########################

SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR})

###############
### COMPILE ###
###############

# COMPILE EXAMPLES
add_executable( avm_main avm_main.cpp setting.cpp util.cpp)

################
###   LINK   ###
################
# FIRST LINK EXAMPLES:
target_link_libraries( avm_main ${OpenCV_LIBS} pthread)

