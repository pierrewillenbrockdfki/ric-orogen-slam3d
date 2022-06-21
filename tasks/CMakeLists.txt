include(slam3dTaskLib)
add_library(${SLAM3D_TASKLIB_NAME} SHARED 
    ${SLAM3D_TASKLIB_SOURCES}
    BaseLogger.cpp
    RockOdometry.cpp
    Common.cpp)

add_dependencies(${SLAM3D_TASKLIB_NAME}
    regen-typekit)

find_package(Boost REQUIRED filesystem)
find_package(PCL 1.7 REQUIRED COMPONENTS io)
target_link_libraries(${SLAM3D_TASKLIB_NAME}
  PUBLIC
    ${OrocosRTT_LIBRARIES}
    ${SLAM3D_TASKLIB_DEPENDENT_LIBRARIES}
    pcl_io)

install(TARGETS ${SLAM3D_TASKLIB_NAME}
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib/orocos)

install(
FILES
    ${SLAM3D_TASKLIB_HEADERS}
    BaseLogger.hpp
    RockOdometry.hpp
    Common.hpp
DESTINATION
    include/orocos/slam3d)
