cmake_minimum_required(VERSION 3.10)

# set the project name
project(Meca500)


set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Release)


# Set library names
set(SOCKET_NAME "socket_comm")
set(MECA500_NAME "robot_interface")



# Set library source files
set(SOCKET_SOURCES "src/socket_comm.cpp")
set(MECA500_SOURCES "src/robot_interface.cpp")



# Create libraries
add_library(${SOCKET_NAME} STATIC ${SOCKET_SOURCES})
add_library(${MECA500_NAME} STATIC ${MECA500_SOURCES})


# Link libraries with include dir
target_include_directories(${SOCKET_NAME} PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_include_directories(${MECA500_NAME} PUBLIC ${CMAKE_SOURCE_DIR}/include)



# Create executables
add_executable(Meca_Connection_Check "examples/Meca_Connection_Check.cpp")
add_executable(Meca_Velo_control "examples/Meca_Velo_control.cpp")
add_executable(Meca_Move "examples/Meca_Move.cpp")


# Link executables with custom libraries
target_link_libraries(Meca_Connection_Check ${SOCKET_NAME})
target_link_libraries(Meca_Connection_Check ${MECA500_NAME})


# Link executables with custom libraries
target_link_libraries(Meca_Velo_control ${SOCKET_NAME})
target_link_libraries(Meca_Velo_control ${MECA500_NAME})


# Link executables with custom libraries
target_link_libraries(Meca_Move ${SOCKET_NAME})
target_link_libraries(Meca_Move ${MECA500_NAME})

