# Target Name

set(DMAP_LIB_NAME DistanceMap)

# Source Files
set(DMAP_SOURCES
    "AbstractMST.cpp"
    "AbstractMST.hpp"
    "DistanceMapCore.cpp"
    "DistanceMapCore.hpp"
    "DistanceMapDLL.hpp"
    "FlowField.cpp"
    "FlowField.hpp"
    "GridToGraph.cpp"
    "GridToGraph.hpp"
    "GridTypes.cpp"
    "GridTypes.hpp"
    "Router.hpp"
    "SparseNavGraph.cpp"
    "SparseNavGraph.hpp"
    "WallDistanceGrid.cpp"
    "WallDistanceGrid.hpp"
)

# Define Static Library
add_library(${DMAP_LIB_NAME} STATIC ${DMAP_SOURCES})

# target_compile_definitions(${DMAP_LIB_NAME} PRIVATE DISTANCEMAP_EXPORTS)

# Include Directories
# Use get_filename_component to resolve the Random path to an absolute path so
# that IDE parsers (clangd) can find headers without needing to normalise ../../../../
get_filename_component(LIBS_RANDOM_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../../Libs/lib/Random" ABSOLUTE)

target_include_directories(${DMAP_LIB_NAME} PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}"    # . (src/core)
    "${CMAKE_CURRENT_SOURCE_DIR}/.." # .. (src)
    "${LIBS_RANDOM_DIR}"             # Resolved path so IDE parsers work
)

# Properties
# set_target_properties(${DMAP_LIB_NAME} PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON)
if(NOT MSVC)
    set_target_properties(${DMAP_LIB_NAME} PROPERTIES PREFIX "")
endif()

# Linking
target_link_libraries(${DMAP_LIB_NAME}
    PRIVATE Algo Stuff Random
    PUBLIC Tracker Util MathStuff
)
