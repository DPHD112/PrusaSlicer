# The system libblosc.so already links zstd; FindBlosc only needs the target name.
if (NOT TARGET zstd::libzstd)
    add_library(zstd::libzstd INTERFACE IMPORTED)
endif()
set(zstd_FOUND TRUE)
