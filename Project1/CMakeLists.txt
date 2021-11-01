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

set(SRC 
${PROJECT_SOURCE_DIR} src/main.cpp
)

add_executable(${PROJECT_NAME} ${SRC})
add_executable(ICP_bags src/icp.cpp)
target_link_libraries(${PROJECT_NAME} 
${PCL_LIBRARIES}
)
target_link_libraries(ICP_bags
${PCL_LIBRARIES}
)
