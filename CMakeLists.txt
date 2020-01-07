cmake_minimum_required(VERSION 3.14.0)
project (GEOPM)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/Modules)

include(cmake/Summary.cmake)
include(cmake/Tools.cmake)
include(cmake/Utilities.cmake)

geopm_project_version()
set(PACKAGE_VERSION VERSION STRING "Package version")

set(CMAKE_INCLUDE_CURRENT_DIR ON)
include_directories(include)
include_directories(src)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 11)

#============================ BUILD ENVIRONMENT ===============================#
if ($ENV{geopm_abi_version})
    set(GEOPM_ABI_VERSION $ENV{geopm_abi_version} CACHE STRING "GEOPM shared object verion")
    message(STATUS "ABI VERSION PRESENT")
else ()
    set(GEOPM_ABI_VERSION "0:0:0" CACHE STRING "GEOPM shared object verion")
    message(STATUS "ABI VERSION ABSENT")
endif ()

set(GEOPM_DEFAULT_PLUGIN_PATH "$ENV{HOME}/build/geopm/lib/geopm" CACHE STRING "GEOPM Default Plugin Path")

add_compile_options(-msse4.2)
add_compile_options(-DGEOPM_DEFAULT_PLUGIN_PATH="${GEOPM_DEFAULT_PLUGIN_PATH}")

set(GEOPM_COVERAGE 0 CACHE BOOL "Enables test coverage reporting")
if (GEOPM_COVERAGE)
    string(APPEND CMAKE_C_FLAGS " --coverage")
    string(APPEND CMAKE_CXX_FLAGS " --coverage")
    string(APPEND CMAKE_LD_FLAGS " --coverage -lgcov")
endif ()

set(GEOPM_OVERHEAD 0 CACHE BOOL "Enables code for measuring overhead")
set(GEOPM_PROCFS 1 CACHE BOOL "Enables use of procfs for process CPU affinity")

set(GEOPM_MSRSAFE_IOCTL_WRITE $ENV{enable_msrsafe_ioctl_write} CACHE BOOL "Enables user of msr-safe ioctl feature for writing (broken as of msr-safe version 1.2.0")

set(ENABLE_BETA $ENV{enable_beta} CACHE BOOL "")
set(ENABLE_OPENMP $ENV{enable_openmp} CACHE BOOL "")
set(ENABLE_OMPT $ENV{enable_ompt} CACHE BOOL "")
set(ENABLE_FORTRAN $ENV{enable_fortran} CACHE BOOL "")
set(ENABLE_DOC 1 CACHE BOOL "")

set(WITH_MPI_BIN "" CACHE STRING "specify directory for mpi compiler wrapper binaries")

#==============================================================================#
# Now that all our flags are set, we can decide which dependencies to search for

include(cmake/Dependencies.cmake)

#==============================================================================#

file(GLOB NON_MPI_SOURCES "src/*.cpp" "src/*.c" "contrib/*/*.cpp")

set(MPI_SOURCES "src/MPIComm.cpp"
                "src/MPICommSplit.cpp"
                "src/geopm_pmpi.c"
                "src/geopm_pmpi_helper.cpp"
                "src/geopm.f90"
                "src/geopm_pmpi_fortran.c"
)

list(TRANSFORM MPI_SOURCES PREPEND "${PROJECT_SOURCE_DIR}/")

set(GEOPM_BENCH_SOURCES "${PROJECT_SOURCE_DIR}/src/ModelApplication.cpp"
                        "${PROJECT_SOURCE_DIR}/src/ModelRegion.cpp"
)

list(REMOVE_ITEM NON_MPI_SOURCES ${MPI_SOURCES})
list(REMOVE_ITEM NON_MPI_SOURCES ${GEOPM_BENCH_SOURCES})

#============================== SOURCE BLACKLISTS =============================#

if (NOT ENABLE_OMPT)
    list(REMOVE_ITEM NON_MPI_SOURCES "${PROJECT_SOURCE_DIR}/src/OMPT.cpp"
                                     "${PROJECT_SOURCE_DIR}/src/ELF.cpp")
endif ()

if (NOT ENABLE_BETA)
    list(REMOVE_ITEM NON_MPI_SOURCES "${PROJECT_SOURCE_DIR}/src/Daemon.cpp"
                                     "${PROJECT_SOURCE_DIR}/src/Daemon.hpp"
                                     "${PROJECT_SOURCE_DIR}/src/DaemonImp.hpp"
                                     "${PROJECT_SOURCE_DIR}/src/PolicyStore.cpp"
                                     "${PROJECT_SOURCE_DIR}/src/PolicyStore.hpp"
                                     "${PROJECT_SOURCE_DIR}/src/PolicyStoreImp.cpp"
                                     "${PROJECT_SOURCE_DIR}/src/PolicyStoreImp.hpp")
endif ()

if (NOT ENABLE_FORTRAN)
    list(REMOVE_ITEM MPI_SOURCES "${PROJECT_SOURCE_DIR}/src/geopm.f90"
                                 "${PROJECT_SOURCE_DIR}/src/geopm_pmpi_fortran.c")
endif ()

#=================================== BINARIES =================================#

add_library(geopm_lib SHARED ${NON_MPI_SOURCES} ${MPI_SOURCES})
set_target_properties(geopm_lib PROPERTIES OUTPUT_NAME geopm)
target_include_directories(geopm_lib PRIVATE ${MPI_CXX_INCLUDE_PATH})
target_link_libraries(geopm_lib Threads::Threads)
target_link_libraries(geopm_lib dl)
target_link_libraries(geopm_lib rt)
target_link_libraries(geopm_lib MPI::MPI_C)

add_library(geopmpolicy_lib SHARED ${NON_MPI_SOURCES})
set_target_properties(geopmpolicy_lib PROPERTIES OUTPUT_NAME geopmpolicy)
target_link_libraries(geopmpolicy_lib Threads::Threads)
target_link_libraries(geopmpolicy_lib dl)
target_link_libraries(geopmpolicy_lib rt)

add_executable(geopmagent main/geopmagent_main.c)
target_link_libraries(geopmagent geopmpolicy_lib)
add_dependencies(geopmagent geopmpolicy_lib)

add_executable(geopmbench main/geopmbench_main.cpp ${GEOPM_BENCH_SOURCES})
target_link_libraries(geopmbench geopm_lib)
add_dependencies(geopmbench geopm_lib)

add_executable(geopmctl main/geopmctl_main.c)
target_link_libraries(geopmctl geopm_lib)
add_dependencies(geopmctl geopm_lib)

add_executable(geopmread main/geopmread_main.cpp)
target_link_libraries(geopmread geopmpolicy_lib)
add_dependencies(geopmread geopmpolicy_lib)

add_executable(geopmwrite main/geopmwrite_main.cpp)
target_link_libraries(geopmwrite geopmpolicy_lib)
add_dependencies(geopmwrite geopmpolicy_lib)

#==============================================================================#

include(cmake/Googletest_main.cmake)

#==============================================================================#

install(
    TARGETS geopm_lib
            geopmpolicy_lib
            geopmagent
            geopmbench
            geopmctl
            geopmread
            geopmwrite
)

# Installed C headers
install(
    DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# Installed C++ headers
# TODO: move to include folder also?
set(CXX_HEADERS "contrib/json11/json11.hpp"
                 "src/Agent.hpp"
                 "src/Agg.hpp"
                 "src/CircularBuffer.hpp"
                 "src/CpuinfoIOGroup.hpp"
                 "src/EnergyEfficientRegion.hpp"
                 "src/FrequencyGovernor.hpp"
                 "src/Exception.hpp"
                 "src/Helper.hpp"
                 "src/IOGroup.hpp"
                 "src/MSR.hpp"
                 "src/MSRControl.hpp"
                 "src/MSRIO.hpp"
                 "src/MSRIOGroup.hpp"
                 "src/MSRSignal.hpp"
                 "src/PlatformIO.hpp"
                 "src/PlatformTopo.hpp"
                 "src/PluginFactory.hpp"
                 "src/PowerBalancer.hpp"
                 "src/PowerGovernor.hpp"
                 "src/RegionAggregator.hpp"
                 "src/SharedMemory.hpp"
                 "src/SharedMemoryScopedLock.hpp"
                 "src/SharedMemoryUser.hpp"
                 "src/TimeIOGroup.hpp"
)
install(
    FILES ${CXX_HEADERS}
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/geopm"
)

#### TODO: move me
##### DOCS ######

find_program(RONN ronn)
# TODO: skip ronn conversion if ronn not found, and just copy the ronn files instead
if(NOT RONN)
set(ENABLE_DOC 0)
endif(NOT RONN)

if (ENABLE_DOC)

file(GLOB RONN_FILES "ronn/geopm*.ronn")
file(GLOB CXX_RONN_FILES "ronn/GEOPM_CXX_MAN_*.ronn")
set(EXTRA_RONN "ronn/header.txt"
               "ronn/index.txt")
list(TRANSFORM RONN_FILES REPLACE "^/.*/|.ronn" "" OUTPUT_VARIABLE MAN_PAGES)
list(TRANSFORM MAN_PAGES PREPEND "ronn/")

message(STATUS ${RONN_FILES})
message(STATUS ${CXX_RONN_FILES})
message(STATUS ${MAN_PAGES})

## Q: one is source, other is in binary dir?? should we put full paths?
configure_file(ronn/geopm.7.ronn.in ronn/geopm.7.ronn)

foreach(ff ${RONN_FILES})
    configure_file(${ff} ronn/${basename} COPYONLY)
endforeach()



## build documentation
add_custom_target(docs ALL
    DEPENDS ${MAN_PAGES}
)
# TODO: removed date from ronn command
# --date=`date "+%Y-%m-%d"`
# TODO: doesn't work :(
#execute_process(COMMAND date "+%Y-%m-%d" OUTPUT_VARIABLE DATE)
set(DATE "2020-01-08")

# TODO: could be a function
foreach(PAGE ${MAN_PAGES})
add_custom_command(
    OUTPUT ${PAGE}
    DEPENDS ${PAGE}.ronn
    COMMAND cd ${CMAKE_CURRENT_BINARY_DIR}/ronn && ${RONN} --date=${DATE} --organization="GEOPM ${VERSION}" ${PAGE}.ronn
    COMMAND cat ${CMAKE_CURRENT_SOURCE_DIR}/ronn/header.txt ${CMAKE_CURRENT_BINARY_DIR}/${PAGE} > ${CMAKE_CURRENT_BINARY_DIR}/${PAGE}.tmp
    COMMAND mv ${CMAKE_CURRENT_BINARY_DIR}/${PAGE}.tmp ${CMAKE_CURRENT_BINARY_DIR}/${PAGE}
    VERBATIM
)

get_filename_component(CHAPTER ${PAGE} EXT)

# TODO: should be gz?
install(
    FILES ${PAGE}
    DESTINATION "${CMAKE_INSTALL_PREFIX}/share/man/man${CHAPTER}"
)

endforeach()

endif(ENABLE_DOC)

#==============================================================================#

configure_file(config.h.in config.h)
configure_file(dox-public/Doxyfile.in dox-public/Doxyfile)
configure_file(geopm-ohpc.spec.in geopm-ohpc.spec)
configure_file(geopm-theta.spec.in geopm-theta.spec)
configure_file(geopm.spec.in geopm.spec)

configure_file(scripts/MANIFEST.in scripts/MANIFEST)
configure_file(scripts/geopmpy/version.py.in scripts/geopmpy/version.py)

#==============================================================================#

geopm_print_configuration_summary()
