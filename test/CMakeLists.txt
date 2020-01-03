include_directories("${PROJECT_SOURCE_DIR}")

include(GoogleTest)

configure_file(../config.h.in config.h)

file(GLOB TEST_SOURCES "*.cpp")

#=============================== TEST BLACKLISTS ==============================#

list(REMOVE_ITEM TEST_SOURCES "${PROJECT_SOURCE_DIR}/test/MPIInterfaceTest.cpp")

if (NOT ENABLE_OMPT)
    list(REMOVE_ITEM TEST_SOURCES "${PROJECT_SOURCE_DIR}/test/ELFTest.cpp")
endif ()

if (NOT ENABLE_BETA)
    list(REMOVE_ITEM TEST_SOURCES "${PROJECT_SOURCE_DIR}/test/DaemonTest.cpp"
                                  "${PROJECT_SOURCE_DIR}/test/PolicyStoreImpTest.cpp")
endif ()

#================================= TEST TARGETS ===============================#

add_executable(geopm_test ${TEST_SOURCES})
target_link_libraries(geopm_test gtest gmock_main)
target_link_libraries(geopm_test geopmpolicy_lib)
gtest_discover_tests(geopm_test)

add_executable(geopm_mpi_test "geopm_test.cpp" "MPIInterfaceTest.cpp")
target_link_libraries(geopm_mpi_test gtest gmock_main geopmpolicy_lib)
gtest_discover_tests(geopm_mpi_test)
