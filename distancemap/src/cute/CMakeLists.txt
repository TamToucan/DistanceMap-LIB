
add_library(CuteDistanceMap SHARED)

target_sources(CuteDistanceMap PRIVATE
	CuteDistanceMap.cpp
)

target_include_directories(CuteDistanceMap PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_include_directories(CuteDistanceMap PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../core")

target_link_libraries(CuteDistanceMap
	PRIVATE DistanceMap
    PRIVATE Random
    PRIVATE MathStuff
    PRIVATE Tracker
    PRIVATE cute_lib
)

set_target_properties(CuteDistanceMap PROPERTIES OUTPUT_NAME CuteDistanceMap)
set_target_properties(CuteDistanceMap PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON)
if(NOT MSVC)
	set_target_properties(CuteDistanceMap PROPERTIES PREFIX "")
endif()

# Windows: Copy dependencies next to CuteDistanceMap so tests can run
add_custom_command(TARGET CuteDistanceMap POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    $<TARGET_FILE:DistanceMap>
    $<TARGET_FILE_DIR:CuteDistanceMap>
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    $<TARGET_FILE:Tracker>
    $<TARGET_FILE_DIR:CuteDistanceMap>
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    $<TARGET_FILE:cute>
    $<TARGET_FILE_DIR:CuteDistanceMap>
)





