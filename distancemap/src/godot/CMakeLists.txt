set(GDDMAP_LIB_NAME GDDistanceMap)

file(GLOB_RECURSE GDDMAP_SOURCES CONFIGURE_DEPENDS
    "*.h" "*.hpp" "*.cpp"
)

add_library(${GDDMAP_LIB_NAME} SHARED ${GDDMAP_SOURCES})
target_compile_definitions(${GDDMAP_LIB_NAME} PRIVATE GDDISTANCEMAP_EXPORTS)

target_include_directories(${GDDMAP_LIB_NAME} PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/../core"
    "${CMAKE_BINARY_DIR}/_deps/libs-src/lib/Util"
)

if(NOT MSVC)
    set_target_properties(${GDDMAP_LIB_NAME} PROPERTIES PREFIX "")
endif()

# Link to the sibling 'DistanceMap' target
target_link_libraries(${GDDMAP_LIB_NAME}
    PRIVATE DistanceMap GDCave Cave GDTracker
    PUBLIC godot::cpp
)

# Copy DLLs
#
set(GD_DMAP_ADDON_PATH, "D:/git/the-lott/addons/gddistancemap/bin")
# Define the path once for readability, but use the explicit string in the command 
# if you want to be 100% sure it isn't lost during build-system generation.
set(TARGET_DEST_DIR "D:/git/the-lott/addons/gddistancemap/bin")

# Ensure the directory exists
add_custom_command(
    TARGET ${GDDMAP_LIB_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "$<TARGET_FILE:DistanceMap>"
        "D:/git/the-lott/addons/gddistancemap/bin"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "$<TARGET_FILE:Tracker>"
        "D:/git/the-lott/addons/gddistancemap/bin"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "$<TARGET_FILE:GDTracker>"
        "D:/git/the-lott/addons/gddistancemap/bin"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "$<TARGET_FILE:${GDDMAP_LIB_NAME}>"
        "D:/git/the-lott/addons/gddistancemap/bin"
    COMMENT "Deploying Lott-Libs to Godot: D:/git/the-lott/addons/gddistancemap/bin"
    VERBATIM
)
