find_package(OpenMP)
# TODO use the new idiom? target link?
if (OPENMP_FOUND)
    string(APPEND CMAKE_C_FLAGS " ${OpenMP_C_FLAGS}")
    string(APPEND CMAKE_CXX_FLAGS " ${OpenMP_CXX_FLAGS}")
    string(APPEND CMAKE_EXE_LINKER_FLAGS " ${OpenMP_EXE_LINKER_FLAGS}")
endif(OPENMP_FOUND)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED THREADS_PREFER_PTHREAD_FLAG)

find_package(MPI REQUIRED)

if (MPI_CXX_VERSION VERSION_GREATER 3)
    set(GEOPM_MPI_CONST "const" CACHE STRING "MPI interfaces use const qualifier.")
    set(GEOPM_MPI_CONST_CAST "" CACHE STRING "MPI interfaces use const qualifier.")
    set(GEOPM_ENABLE_MPI3 1 CACHE BOOL "MPI-3 support enabled.")
else (MPI_CXX_VERSION VERSION_GREATER 3)
    set(GEOPM_MPI_CONST "" CACHE STRING "MPI interfaces do not use const qualifier.")
    set(GEOPM_MPI_CONST_CAST "const_cast<t>" CACHE STRING "MPI interfaces do not use const qualifier.")
    set(GEOPM_ENABLE_MPI3 1 CACHE BOOL "MPI-3 support enabled.")
endif (MPI_CXX_VERSION VERSION_GREATER 3)

if (ENABLE_BETA)
    find_package (SQLite3 REQUIRED)
    include_directories(${SQLITE3_INCLUDE_DIRS})
    link_libraries(${SQLITE3_LIBRARIES})
endif (ENABLE_BETA)
