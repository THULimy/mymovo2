cmake_minimum_required(VERSION 2.8)
set(OpenCV_DIR "/home/limy/opencv-3.0.0/build")
project( mono-vo2 )
find_package( OpenCV 3.0 REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} )

file(GLOB viso
    "src/vo_features.h"
    "src/visodo.cpp"
)

file(GLOB noisedata
    "src/GPSnoise.cpp"
)
add_executable( vo ${viso} )
target_link_libraries( vo ${OpenCV_LIBS} )

add_executable( getnd ${noisedata} )
target_link_libraries( getnd ${OpenCV_LIBS} )