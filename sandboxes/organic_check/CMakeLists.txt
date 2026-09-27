add_executable(organic_check
    organic_check.cpp
    ${CMAKE_SOURCE_DIR}/tests/fff_print/test_data.cpp
)
target_include_directories(organic_check PRIVATE ${CMAKE_SOURCE_DIR}/tests/fff_print)
target_link_libraries(organic_check test_common slic3r-shared)
