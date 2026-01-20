# --- 1. Test Executable (Functional Tests) ---
add_executable(dmapTest testMain.cpp)

target_include_directories(dmapTest PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../src")
target_link_libraries(dmapTest PRIVATE DistanceMapLib::DistanceMap)

# Windows: Copy DLL
if(TARGET DistanceMapLib::DistanceMap)
    add_custom_command(TARGET dmapTest POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:DistanceMapLib::DistanceMap>
        $<TARGET_FILE:Util>
        "${CMAKE_CURRENT_SOURCE_DIR}/GRID.txt"
        $<TARGET_FILE_DIR:dmapTest>
    )
endif()


# --- 2. Time Executable (Benchmarks) ---
add_executable(dmapTime timeMain.cpp)

target_include_directories(dmapTime PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../src")
target_link_libraries(dmapTime PRIVATE DistanceMapLib::DistanceMap)

# Windows: Copy DLL (Same logic for the second exe)
if(TARGET DistanceMapLib::DistanceMap)
    add_custom_command(TARGET dmapTime POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:DistanceMapLib::DistanceMap>
        $<TARGET_FILE:Util>
        $<TARGET_FILE_DIR:dmapTime>
    )
endif()

