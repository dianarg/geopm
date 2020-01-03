set(LINK_WHAT_YOU_USE 1 CACHE BOOL "Enables link what you use")

if (LINK_WHAT_YOU_USE)
    message(STATUS "Enabling link-what-you-use - ok")
    set (CMAKE_LINK_WHAT_YOU_USE ON)
endif (LINK_WHAT_YOU_USE)

set(INCLUDE_WHAT_YOU_USE 1 CACHE BOOL "Enables include what you use")

if (INCLUDE_WHAT_YOU_USE)
    find_program(INCLUDE_WHAT_YOU_USE_PATH NAMES include-what-you-use iwyu)
    if (INCLUDE_WHAT_YOU_USE_PATH)
        message(STATUS "Looking for include-what-you-use - found")
        set (INCLUDE_WHAT_YOU_USE_OPTS
            ${INCLUDE_WHAT_YOU_USE_PATH}
            "-Xiwyu"
            "--mapping_file=${PROJECT_SOURCE_DIR}/iwyu.imp"
            )
        set (CMAKE_C_INCLUDE_WHAT_YOU_USE "${INCLUDE_WHAT_YOU_USE_OPTS}")
        set (CMAKE_CXX_INCLUDE_WHAT_YOU_USE "${INCLUDE_WHAT_YOU_USE_OPTS}")
    else ()
        message(STATUS "Looking for include-what-you-use - not found")
    endif ()
endif ()

set(CLANG_TIDY 1 CACHE BOOL "Enables clang-tidy")

if (CLANG_TIDY)
    find_program(CLANG_TIDY_PATH NAMES clang-tidy)
    if (CLANG_TIDY_PATH)
        message(STATUS "Looking for clang-tidy - found")
        set (CLANG_TIDY_OPTS
            ${CLANG_TIDY_PATH}
            # "-fix"
            "-checks=-*,readability-*,modernize-*"
            )
        set (CMAKE_C_CLANG_TIDY "${CLANG_TIDY_OPTS}")
        set (CMAKE_CXX_CLANG_TIDY "${CLANG_TIDY_OPTS}")
    else ()
        message(STATUS "Looking for clang-tidy - not found")
    endif ()
endif ()
