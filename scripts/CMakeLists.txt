#  Copyright (c) 2015, 2016, 2017, 2018, Intel Corporation
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#      * Redistributions of source code must retain the above copyright
#        notice, this list of conditions and the following disclaimer.
#
#      * Redistributions in binary form must reproduce the above copyright
#        notice, this list of conditions and the following disclaimer in
#        the documentation and/or other materials provided with the
#        distribution.
#
#      * Neither the name of Intel Corporation nor the names of its
#        contributors may be used to endorse or promote products derived
#        from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
#  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY LOG OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
find_package (Python REQUIRED)

find_program(Awk_EXECUTABLE NAMES awk gawk
             DOC "awk utility")

if (NOT Awk_EXECUTABLE)
	message(FATAL_ERROR "awk is required for test discovery")
endif()

set(BUILD_TS "${CMAKE_CURRENT_BINARY_DIR}/build_timestamp")
set(DISTTOOLS_BUILD_DIR "${CMAKE_CURRENT_BINARY_DIR}/build")

configure_file(geopmpy/version.py.in geopmpy/version.py)

file(GLOB python_test_files ${CMAKE_CURRENT_SOURCE_DIR}/test/Test*.py)
file(GLOB python_source_files ${CMAKE_CURRENT_SOURCE_DIR}/geopmpy/*.py)
set(python_scripts
        geopmanalysis
        geopmconvertreport
        geopmlaunch
        geopmplotter)

# Generate the pyc files and a dummy timestamp file. The timestamp file
# is used for tracking whether the install command is up to date with the
# latest build.
add_custom_command(OUTPUT ${BUILD_TS}
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        DEPENDS ${python_test_files} ${python_source_files} ${python_scripts}
        COMMAND ${Python_EXECUTABLE} setup.py build --build-base ${DISTTOOLS_BUILD_DIR}
        COMMAND ${CMAKE_COMMAND} -E touch ${BUILD_TS})

add_custom_target(geopm-python ALL
        DEPENDS ${BUILD_TS})

enable_testing()

set(ctest_tests_file "${CMAKE_CURRENT_BINARY_DIR}/geopm_python_tests.cmake")

# Whenever geopm-python is updated, update the list of tests
add_custom_command(
        TARGET geopm-python POST_BUILD
        BYPRODUCTS "${ctest_tests_file}"
        DEPENDS "${PROJECT_SOURCE_DIR}/cmake/DiscoverPythonTests.cmake"
        COMMAND "${CMAKE_COMMAND}"
        -D "Awk_EXECUTABLE=${Awk_EXECUTABLE}"
                -D "Python_EXECUTABLE=${Python_EXECUTABLE}"
                -D "PYTHON_TEST_FILES=${python_test_files}"
                -D "PYTHON_WORKING_DIRECTORY=${CMAKE_CURRENT_SOURCE_DIR}"
                -D "TEST_PREFIX=geopmpy."
                -D "CTEST_FILE=${ctest_tests_file}"
                -P "${PROJECT_SOURCE_DIR}/cmake/DiscoverPythonTests.cmake"
        VERBATIM)

set_property(DIRECTORY
             APPEND PROPERTY TEST_INCLUDE_FILES "${ctest_tests_file}")

install(CODE "execute_process(
                COMMAND ${Python_EXECUTABLE} setup.py install -O1 --root ${CMAKE_INSTALL_PREFIX} --prefix /usr
                WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})")
