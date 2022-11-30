cmake_minimum_required(VERSION 3.0)
add_compile_options( -g )

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(Auto)

add_subdirectory(lib)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

include_directories(
    ${OpenCV_INCLUDE_DIRS} 
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/

    # sdk library header 
    ${CMAKE_SOURCE_DIR}/lib/sdk/include
    ${CMAKE_SOURCE_DIR}/lib/sdk/src
    ${CMAKE_SOURCE_DIR}/lib/sdk/src/hal

    # PCL LIB
    ${PCL_INCLUDE_DIRS}
)



if ( APPLE )
    message("Your OS is MaxOS")
    include_directories(${CMAKE_SOURCE_DIR}/lib/sdk/src/macOS)
elseif ( UNIX ) 
    message("Your OS is UINX")
    include_directories(${CMAKE_SOURCE_DIR}/lib/sdk/src/linux)
endif()

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(${PROJECT_NAME} 
    src/main.cpp
    src/Control.cpp
    src/DogStatus.cpp
    src/System.cpp
    src/Lidar.cpp
)

target_link_libraries(${PROJECT_NAME} 
    ${OpenCV_LIBS} 
    SDK_LIBRARY
    -lpthread
    ${PCL_LIBRARIES}
)


