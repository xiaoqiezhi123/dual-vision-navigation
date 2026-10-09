cmake_minimum_required(VERSION 3.0)

set(CMAKE_CXX_STANDARD 11)
project(Gait_HLIP)
message(STATUS "Project source directory: ${PROJECT_SOURCE_DIR}")

include_directories(${PROJECT_SOURCE_DIR}/../)

include_directories(${PROJECT_SOURCE_DIR}/../DataPackage/include)
include_directories(${PROJECT_SOURCE_DIR}/Joystick/include)
include_directories(${PROJECT_SOURCE_DIR}/include)

aux_source_directory(${PROJECT_SOURCE_DIR}/Joystick/src dir_js)

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


add_executable(test ${PROJECT_SOURCE_DIR}/test/test.cpp

${dir_js}
${dir_datapackage}
)



target_link_libraries(test  PUBLIC 

pthread

)

# install(TARGETS DataPackage DESTINATION ${PROJECT_SOURCE_DIR}/lib)

