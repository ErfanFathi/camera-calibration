cmake_minimum_required(VERSION 2.8)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-std=c++14")

project( Calibration )

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable( Calibration
                 main.cpp
                 ChessPatternDetector.h
                 ChessPatternDetector.cpp
                 RotationMatrix.h
                 GeometryParameter.h
                 Projection.h
                 Projection.cpp
                 PSO.h
                 Quater.h
                 Quater.cpp )

target_link_libraries( Calibration ${OpenCV_LIBS} )