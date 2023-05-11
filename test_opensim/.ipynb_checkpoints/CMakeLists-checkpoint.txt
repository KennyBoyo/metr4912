cmake_minimum_required( VERSION 3.10 )

project( test_opensim )

find_package( OpenSim )
find_package( Simbody )

include_directories(${OpenSim_INCLUDE_DIRS})
include_directories(${Simbody_INCLUDE_DIR})

if (Simbody_FOUND)
    message(STATUS "FOUND SIMTK")
endif()

if (OpenSim_FOUND)
message (STATUS "${Simbody_LIBRARIES}")
    add_executable(opensim_test src/opensim-test.cpp)
    add_executable(opensim_import_test src/opensim-import-test.cpp)
    target_link_libraries(opensim_test ${Simbody_LIBRARIES} ${OpenSim_LIBRARIES})
    target_link_libraries(opensim_import_test ${Simbody_LIBRARIES} ${OpenSim_LIBRARIES})
    
endif()