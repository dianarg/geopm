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

# Generate a CMake script that contains add_test calls for each discovered python test.
# This script updater is expected to be update

# Discover the test source files
set(script_lines)
set(all_tests)

# Within each test source file, discover the defined tests
foreach (python_test_file ${PYTHON_TEST_FILES})
        get_filename_component(modulename ${python_test_file} NAME_WE)
        file(READ "${python_test_file}" contents)
        string(REGEX MATCHALL "${gtest_test_type_regex} *\\(([A-Za-z_0-9 ,]+)\\)" found_tests "${contents}")
        execute_process(
                COMMAND ${Awk_EXECUTABLE} -F "([[:space:]]+|\\\\()"
                        "/class Test.*:/ {classname=$2} \
                         /def[[:space:]]+test_/ {printf classname\".\"$3\";\"}"
                        ${python_test_file}
                OUTPUT_VARIABLE discovered_tests
                RESULT_VARIABLE result)

        if (NOT ${result} EQUAL 0)
                message(FATAL_ERROR "Python test discovery failed for ${python_test_file}")
        endif()

        foreach(test_case ${discovered_tests})
                string(PREPEND test_case "${modulename}.")
                string(CONCAT ctest_test_case ${TEST_PREFIX} ${test_case})

                list(APPEND script_lines
                     "add_test(${ctest_test_case}\n"
                     "        \"${Python_EXECUTABLE}\" -m unittest \"test.${test_case}\" --verbose)\n")
                list(APPEND all_tests "${ctest_test_case}\n")
        endforeach()
endforeach()

list(APPEND script_lines
     "set_tests_properties(${all_tests}\n"
     "        PROPERTIES WORKING_DIRECTORY ${PYTHON_WORKING_DIRECTORY})\n")

file(WRITE "${CTEST_FILE}" ${script_lines})
