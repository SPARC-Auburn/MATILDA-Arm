cmake_minimum_required(VERSION 2.8)
project( matilda )
find_package( OpenCV REQUIRED )
add_executable( matilda ObjectTracker.cpp )
target_link_libraries( matilda ${OpenCV_LIBS} )


