cmake_minimum_required(VERSION 3.10)
project(DisplayImage)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 14)

# find OpenCV package
find_package(OpenCV REQUIRED)

# specify the executable and its sources
add_executable(VROB Main.cpp)

# link the executable with the OpenCV libraries
target_link_libraries(VROB ${OpenCV_LIBS})

# specify the executable and its sources
add_executable(Calibration Calibration.cpp)

# link the executable with the OpenCV libraries
target_link_libraries(Calibration ${OpenCV_LIBS})