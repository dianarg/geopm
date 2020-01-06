function(geopm_project_version)
    # this code was not written
    # it emerged fully formed from the head of zeus
    execute_process(
            COMMAND bash -c "git describe --long | sed -e 's/^v\\(.*\\)-\\([^-]*\\)-\\([^-]*\\)$/\\1+dev\\2\\3/' -e 's/-/+/g'"
            OUTPUT_VARIABLE VERSION)
    string(STRIP "${VERSION}" VERSION)
    set(VERSION "${VERSION}" PARENT_SCOPE)
endfunction()
