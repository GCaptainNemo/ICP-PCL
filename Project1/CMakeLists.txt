cmake_minimum_required (VERSION 3.3.1)
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS "/EHsc ${CMAKE_CXX_FLAGS}")

PROJECT("pc_registration")

# PCL
find_package(PCL REQUIRED)


MESSAGE(STATUS "Project: SERVER") 


include_directories(
${PCL_INCLUDE_DIRS}
)

set(DIR_SRC 
${PROJECT_SOURCE_DIR} src/main.cpp src/plane_fitting.cpp
)
set(DIR_HEADERS
${PROJECT_SOURCE_DIR} include/plane_fitting.h
)

set(DIR_SRCS_ICP    src/icp.cpp src/icp_main.cpp)
set(DIR_HEADERS_ICP    include/icp.h)

add_executable(${PROJECT_NAME} ${DIR_SRC} ${DIR_HEADERS})

add_executable(ICP_bags ${DIR_SRCS_ICP} ${DIR_HEADERS_ICP})

target_link_libraries(${PROJECT_NAME} 
${PCL_LIBRARIES}
)

target_link_libraries(ICP_bags
${PCL_LIBRARIES}
)
