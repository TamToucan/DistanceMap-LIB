# Target Name
set(DMAP_LIB_NAME DistanceMap)

# Source Files
file(GLOB_RECURSE DMAP_SOURCES CONFIGURE_DEPENDS
    "*.h" "*.hpp" "*.cpp"
)

# Define Shared Library
add_library(${DMAP_LIB_NAME} SHARED ${DMAP_SOURCES})

target_compile_definitions(${DMAP_LIB_NAME} PRIVATE DISTANCEMAP_EXPORTS)

# Include Directories
target_include_directories(${DMAP_LIB_NAME} PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}"    # . (src/core)
    "${CMAKE_CURRENT_SOURCE_DIR}/.." # .. (src)
    # Deps from Lott-Libs/Libs
    "${CMAKE_BINARY_DIR}/_deps/libs-src/lib/MathStuff"
    "${CMAKE_BINARY_DIR}/_deps/libs-src/lib/Algo"
    "${CMAKE_BINARY_DIR}/_deps/libs-src/lib/Util"
    "${CMAKE_BINARY_DIR}/_deps/libs-src/lib/Stuff"  # <--- ADDED THIS LINE
)

# Properties
set_target_properties(${DMAP_LIB_NAME} PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON)
if(NOT MSVC)
    set_target_properties(${DMAP_LIB_NAME} PROPERTIES PREFIX "")
endif()

# Linking
target_link_libraries(${DMAP_LIB_NAME}
    PRIVATE Algo MathStuff Stuff
    PUBLIC Tracker
)
