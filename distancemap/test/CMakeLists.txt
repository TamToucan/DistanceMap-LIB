# On MinGW, link runtime statically so exes don't depend on PATH-resolved
# libstdc++/libgcc DLLs (e.g. Git's older MinGW in PATH causes ABI mismatch).
if(MINGW)
    add_link_options(-static-libstdc++ -static-libgcc)
endif()

# --- 1. Test Executable (Functional Tests) ---
add_executable(dmapTest testMain.cpp)

target_include_directories(dmapTest PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../src")
target_link_libraries(dmapTest PRIVATE DistanceMapLib::DistanceMap)

add_custom_command(TARGET dmapTest POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    "${CMAKE_CURRENT_SOURCE_DIR}/GRID.txt"
    $<TARGET_FILE_DIR:dmapTest>
)

# --- 2. Time Executable (Benchmarks) ---
add_executable(dmapTime timeMain.cpp)

target_include_directories(dmapTime PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../src")
target_link_libraries(dmapTime PRIVATE DistanceMapLib::DistanceMap)

# --- 3. Room Detection Test ---
add_executable(dmapRoomDetect testRoomDetect.cpp)

target_include_directories(dmapRoomDetect PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../src")
target_link_libraries(dmapRoomDetect PRIVATE DistanceMapLib::DistanceMap)

add_custom_command(TARGET dmapRoomDetect POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    "${CMAKE_CURRENT_SOURCE_DIR}/GRID.txt"
    $<TARGET_FILE_DIR:dmapRoomDetect>
)

