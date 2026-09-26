# Headless build of PrusaSlicer's slicing libraries plus slice_stl, from system packages.
# The full build needs the GUI (src/CMakeLists.txt refuses SLIC3R_GUI=OFF), and its
# dependencies need downloads that some sandboxes block. See ../README.md.
cmake_minimum_required(VERSION 3.20)
project(slic3r_headless C CXX)

get_filename_component(_ps_default "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
set(PS "${_ps_default}" CACHE PATH "PrusaSlicer source tree")
set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
list(APPEND CMAKE_MODULE_PATH ${PS}/cmake/modules/)
list(APPEND CMAKE_PREFIX_PATH ${CMAKE_CURRENT_LIST_DIR}/stubs)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif ()

set(SLIC3R_STATIC OFF)
set(SLIC3R_PCH OFF)
set(SLIC3R_BUILD_TESTS OFF)
set(SLIC3R_GUI OFF)
set(SLIC3R_ENABLE_WEBKIT OFF)
set(SLIC3R_ENABLE_FORMAT_STEP OFF)
set(SLIC3R_ENABLE_PROFILING OFF)
set(SLIC3R_RESOURCES_DIR ${PS}/resources)

include(${PS}/version.inc)
include(GNUInstallDirs)
include(Utils)
include(DoxygenTarget)

add_compile_definitions(wxUSE_UNICODE _UNICODE UNICODE WXINTL_NO_GETTEXT_MACRO wxNO_UNSAFE_WXSTRING_CONV)
add_compile_options(-Wno-unknown-pragmas -Wno-deprecated-declarations $<$<COMPILE_LANGUAGE:CXX>:-Wno-reorder>
    -Werror=return-type -Wno-ignored-attributes -fsigned-char)

# Shims for older system packages (Ubuntu 24.04), written to the build directory.
set(_shim ${CMAKE_BINARY_DIR}/shim/include)
# Qhull 2020.2 declares template constructors as QhullSet<T>(...), which C++20 rejects.
if (EXISTS /usr/include/libqhullcpp/QhullSet.h)
    file(COPY /usr/include/libqhullcpp DESTINATION ${_shim})
    foreach (_header QhullLinkedList.h QhullSet.h)
        file(READ /usr/include/libqhullcpp/${_header} _text)
        string(REGEX REPLACE "([ \t]~?)(QhullLinkedList|QhullSet|QhullSetIterator)<T>\\(" "\\1\\2(" _text "${_text}")
        file(WRITE ${_shim}/libqhullcpp/${_header} "${_text}")
    endforeach ()
endif ()
# spdlog before 1.14 has no stopwatch::elapsed_ms().
if (EXISTS /usr/include/spdlog/stopwatch.h)
    file(READ /usr/include/spdlog/stopwatch.h _text)
    if (NOT _text MATCHES "elapsed_ms")
        string(REPLACE "    void reset()"
"    std::chrono::milliseconds elapsed_ms() const
    {
        return std::chrono::duration_cast<std::chrono::milliseconds>(clock::now() - start_tp_);
    }

    void reset()" _text "${_text}")
        file(WRITE ${_shim}/spdlog/stopwatch.h "${_text}")
    endif ()
endif ()
# magic_enum before 0.9.6 installs its headers without the magic_enum/ directory.
if (EXISTS /usr/include/magic_enum.hpp AND NOT EXISTS /usr/include/magic_enum/magic_enum.hpp)
    file(WRITE ${_shim}/magic_enum/magic_enum.hpp "#pragma once\n#include <magic_enum.hpp>\n")
endif ()
include_directories(BEFORE SYSTEM ${_shim})

# cmake/modules/GlobalDependencies.cmake without the GUI and Tracy.
include(Boost)
find_package(TBB REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(NLopt 1.4 REQUIRED)
find_package(EXPAT REQUIRED)
add_library(libexpat INTERFACE)
if (TARGET EXPAT::EXPAT)
    target_link_libraries(libexpat INTERFACE EXPAT::EXPAT)
elseif (TARGET expat::expat)
    target_link_libraries(libexpat INTERFACE expat::expat)
else ()
    target_link_libraries(libexpat INTERFACE ${EXPAT_LIBRARIES})
endif ()
find_package(PNG REQUIRED)
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
find_package(cereal 1.3.2 REQUIRED)
add_library(libcereal INTERFACE)
if (NOT TARGET cereal::cereal)
    target_link_libraries(libcereal INTERFACE cereal)
else ()
    target_link_libraries(libcereal INTERFACE cereal::cereal)
endif ()
find_package(fmt REQUIRED)

add_subdirectory(${PS}/bundled_deps bundled_deps)
foreach (dir slic3r-base slic3r-domain slic3r-biz-algorithms slic3r-biz-cgal-algorithms slic3r-biz-parser
             slic3r-gcode-reader libpgcode slic3r-jthread libslic3r slic3r-test-utils)
    add_subdirectory(${PS}/src/${dir} src/${dir})
endforeach ()

add_executable(slice_stl slice_stl.cpp)
target_include_directories(slice_stl PRIVATE ${PS}/src/libslic3r/src)
target_link_libraries(slice_stl PRIVATE libslic3r slic3r-biz-algorithms slic3r-test-utils)
