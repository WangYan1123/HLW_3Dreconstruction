cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(3Dconstruction LANGUAGES CXX)

if (MSVC) # 解决中文乱码
    message(STATUS "check utf-8")
    string(APPEND CMAKE_CXX_FLAGS " /source-charset:utf-8 /execution-charset:utf-8")
endif ()

# QT
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(Qt5_DIR "D:/wy-project/thirdparty/Qt5.14.0/5.14.0/msvc2017_64/lib/cmake/Qt5")
message(STATUS "Qt5_DIR is set to ${Qt5_DIR}")
find_package(Qt5 COMPONENTS Core Widgets REQUIRED)

# OPENCV
set(OpenCV_DIR "D:/wy-project/thirdparty/opencv4.5.4/opencv/build")
message(STATUS "OpenCV is set to ${OpenCV_DIR}")
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# PCL
set(PCL_DIR "D:/wy-project/thirdparty/PCL 1.11.1/cmake")
set(PCL_ROOT "${PROJECT_SOURCE_DIR}/thirdparty/PCL 1.11.1")
message(STATUS "PCL_DIR is set to ${PCL_DIR}")
if (POLICY CMP0074) # PCL一个小warning
    cmake_policy(SET CMP0074 NEW)
endif ()
find_package(PCL REQUIRED)

if(PCL_FOUND)
    include_directories(${PCL_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS})
    message(STATUS "Found PCL version: ${PCL_VERSION}.")
else()
    message(STATUS "PCL is not found.")
endif()

# Open3D
set(Open3D_ROOT "D:/wy-project/thirdparty/open3d-0.16.0")
message(STATUS "Open3D is set to ${Open3D_ROOT}")
find_package(Open3D REQUIRED)
include_directories("${Open3D_ROOT}/include/open3d/3rdparty" "${Open3D_ROOT}/include")
option(STATIC_WINDOWS_RUN
"Use static (MT/MTd) Windows runtime" ON)
if (STATIC_WINDOWS_RUNTIME)
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
else ()
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
endif ()
add_compile_definitions(NOMINMAX) # 重要，open3d 源码中的min max 报错

include_directories(
    cole  
    registration
    segmentation
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

add_subdirectory(registration)  
add_subdirectory(segmentation)
add_subdirectory(cole)

add_definitions(-DROOT_PATH="${PROJECT_SOURCE_DIR}")

file(GLOB SOURCES "construction.cpp" "construction.h")


add_executable(3Dconstruction ${SOURCES})
target_link_libraries(3Dconstruction
     PRIVATE
     cole  
     registration
     segmentation
     Qt5::Core
     Qt5::Widgets
     ${OpenCV_LIBS}
     ${PCL_LIBRARIES}
     Open3D::Open3D
     )


# 自动添加需要的dll文件
if(WIN32)
    file(GLOB PCL_DLLS "${PCL_ROOT}/bin/*.dll")
    
    foreach(PCL_DLL ${PCL_DLLS})
        add_custom_command(TARGET 3Dconstruction POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy
            "${PCL_DLL}"
            "$<TARGET_FILE_DIR:3Dconstruction>")
    endforeach()

    file(GLOB VTK_DLLS "${PCL_ROOT}/3rdParty/VTK/bin/*.dll")
    foreach(VTK_DLL ${VTK_DLLS})
        add_custom_command(TARGET 3Dconstruction POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy
            "${VTK_DLL}"
            "$<TARGET_FILE_DIR:3Dconstruction>")
    endforeach()

    file(GLOB OpenNI_DLLS "${PCL_ROOT}/3rdParty/OpenNI2/Redist/*.dll")
    foreach(OpenNI_DLLS ${OpenNI_DLLS})
        add_custom_command(TARGET 3Dconstruction POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy
            "${OpenNI_DLLS}"
            "$<TARGET_FILE_DIR:3Dconstruction>")
    endforeach()
endif()