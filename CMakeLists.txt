CMAKE_MINIMUM_REQUIRED(VERSION 3.10)
project("coursedemo")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
#SET(eigenbasic eigenbasic.cpp)
#MESSAGE(STATUS "This is BINARY dir " ${testEigenlib_BINARY_DIR})
############Pangolin_FOUND#############
find_package(Pangolin REQUIRED)
IF(Pangolin_FOUND)
    message("Pangolin_INCLUDE_DIRS  is found:" ${Pangolin_DIR})
    message("Pangolin_LIBRARIES  is found:" ${Pangolin_LIBRARIES})
ENDIF(Pangolin_FOUND)

########Opencv Lib#####################
find_package(OpenCV REQUIRED)
IF(OpenCV_FOUND)
    message("OpenCV_INCLUDE_DIRS  is found:" ${OpenCV_INCLUDE_DIRS})
    message("OpenCV_LIBRARIES  is found:" ${OpenCV_LIBS})
ENDIF(OpenCV_FOUND)
add_definitions(${OpenCV_DEFINITIONS})
##############Sophus-1.22.4_FOUND#############
#find_package(Sophus REQUIRED)
#IF(Sophus_FOUND)
#    message("Sophus_INCLUDE_DIRS  is found:" ${Sophus_DIR})
#    message("Sophus_LIBRARIES  is found:" ${Sophus_LIBRARIES})
#ENDIF(Sophus_FOUND)

include_directories(extern/eigen-3.4.0
                    extern/Sophus-1.22.10
                    ${OpenCV_INCLUDE_DIRS}
                    ${Pangolin_DIR}
       )
find_package(fmt)
#find_package(Sophus REQUIRED)

#MESSAGE("Sophus_LIBRARIES:" ${CMAKE_SOURCE_DIR}/extern/)

add_executable(eigenbasic eigenbasic.cpp)

add_executable(eigenGeometry eigenGeometry.cpp)

add_executable(useSophus useSophus.cpp)
target_link_libraries(useSophus fmt::fmt-header-only)


add_executable(feature_extraction feature_extraction.cpp)
target_link_libraries( feature_extraction ${OpenCV_LIBS})

add_executable(featuretime featuretime.cpp)
target_link_libraries( featuretime ${OpenCV_LIBRARIES})

add_executable(draw_trajectory draw_trajectory.cpp)
target_link_libraries(draw_trajectory ${OpenCV_LIBRARIES} ${Pangolin_LIBRARIES} fmt::fmt-header-only)

add_executable(pose_estimation_2d2d pose_estimation_2d2d.cpp)
target_link_libraries(pose_estimation_2d2d ${OpenCV_LIBRARIES} ${Pangolin_LIBRARIES})

add_executable(triangulation triangulation.cpp)
target_link_libraries(triangulation ${OpenCV_LIBRARIES} ${Pangolin_LIBRARIES})

