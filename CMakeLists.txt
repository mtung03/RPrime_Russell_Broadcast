CMAKE_MINIMUM_REQUIRED(VERSION 2.4)
PROJECT(russBcast)

if(COMMAND cmake_policy)
	cmake_policy(SET CMP0003 OLD)
	cmake_policy(SET CMP0015 OLD)
endif(COMMAND cmake_policy)

SET(EXECUTABLE_OUTPUT_PATH ".")

find_package(ZED 2 REQUIRED)
find_package(CUDA ${ZED_CUDA_VERSION} REQUIRED)
find_package( OpenCV REQUIRED )

include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${ZED_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

link_directories(${ZED_LIBRARY_DIR})
link_directories(${CUDA_LIBRARY_DIRS})
#link_directories(${OpenCV_LIBS})

ADD_EXECUTABLE(russBcast "russ_broadcast.cpp")

#add_definitions(-std=c++11 -g -O3 -pthread)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -O3 -pthread")

TARGET_LINK_LIBRARIES(russBcast 
			${ZED_LIBRARIES}
			${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY}
${CUDA_NPP_LIBRARIES_ZED} ${OpenCV_LIBS} )

