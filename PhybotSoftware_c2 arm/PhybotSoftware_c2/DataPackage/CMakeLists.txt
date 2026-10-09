cmake_minimum_required(VERSION 3.0)

set(CMAKE_CXX_STANDARD 11)
project(DataPackage)
message(STATUS "Project source directory: ${PROJECT_SOURCE_DIR}/..")
include_directories(${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/../ThirdParty)
include_directories(${PROJECT_SOURCE_DIR}/../ThirdParty/eigen3)

include_directories(${PROJECT_SOURCE_DIR}/../ThirdParty/pinocchio/include)
include_directories(${PROJECT_SOURCE_DIR}/../ThirdParty/yaml/include)
include_directories(${PROJECT_SOURCE_DIR}/../ThirdParty/urdfdom/include)


link_directories(${PROJECT_SOURCE_DIR}/../ThirdParty/pinocchio/lib)
link_directories(${PROJECT_SOURCE_DIR}/../ThirdParty/yaml/lib)
link_directories(${PROJECT_SOURCE_DIR}/../ThirdParty/urdfdom/lib)
link_directories(${PROJECT_SOURCE_DIR}/../ThirdParty/boost/lib)


aux_source_directory(${PROJECT_SOURCE_DIR}/src dir_datapackage)
# add_library(DataPackage SHARED ${dir_datapackage})

# target_link_libraries(DataPackage pinocchio yaml-cpp)

add_executable(main ${PROJECT_SOURCE_DIR}/test/main.cpp
    ${PROJECT_SOURCE_DIR}/src/DataPackage.cpp)

target_link_libraries(main  PUBLIC 
pinocchio_default 
pinocchio_parsers
yaml-cpp 
urdfdom_model )

# install(TARGETS DataPackage DESTINATION ${PROJECT_SOURCE_DIR}/lib)

