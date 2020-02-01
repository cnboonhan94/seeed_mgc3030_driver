cmake_minimum_required(VERSION 2.8.3)
project(seeed_mgc3030_driver)

find_package(catkin REQUIRED)
file(GLOB SRC_FILES 
    ${CMAKE_CURRENT_SOURCE_DIR}/external/Seeed_mgc3x30/src/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/external/Seeed_mgc3x30/src/*.h
    )

catkin_package()

add_executable(main src/main.c ${SRC_FILES}) 
target_include_directories(main PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/external/Seeed_mgc3x30>
    )
target_link_libraries(main 
    wiringPi 
    curses
    crypt
    pthread
    rt
    m
    )

add_executable(mgc3030_node src/mgc3030_node.cpp ${SRC_FILES}) 
target_include_directories(mgc3030_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/external/Seeed_mgc3x30>
    )
target_link_libraries(mgc3030_node 
    wiringPi 
    curses
    crypt
    pthread
    rt
    m
    )

install(TARGETS main mgc3030_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

