aux_source_directory(. DIR_SRC)

add_subdirectory(motionPlanner)
add_executable(motion ${DIR_SRC})
target_link_libraries(motion motion_planner_lib)

SET( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin) 

