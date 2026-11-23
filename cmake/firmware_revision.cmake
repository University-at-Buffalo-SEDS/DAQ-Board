# Short git revision for the SEDS-SD format block (Core/Inc/sd_format.h).
# Informational only: SD readers never accept or refuse a file by it, so the
# build works without git (fallback "unknown"). Evaluated at configure time;
# a new commit or checkout re-runs configure through the HEAD/ref dependency.
find_package(Git QUIET)
set(SEDS_FIRMWARE_REVISION "unknown")
if(GIT_FOUND)
    execute_process(COMMAND "${GIT_EXECUTABLE}" rev-parse --short=12 HEAD
        WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
        OUTPUT_VARIABLE _seds_revision OUTPUT_STRIP_TRAILING_WHITESPACE
        RESULT_VARIABLE _seds_revision_result ERROR_QUIET)
    if(_seds_revision_result EQUAL 0 AND _seds_revision MATCHES "^[0-9a-f]+$")
        set(SEDS_FIRMWARE_REVISION "${_seds_revision}")
        execute_process(COMMAND "${GIT_EXECUTABLE}" status --porcelain --untracked-files=no
            WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
            OUTPUT_VARIABLE _seds_dirty OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
        if(NOT _seds_dirty STREQUAL "")
            string(APPEND SEDS_FIRMWARE_REVISION "-dirty")
        endif()
        execute_process(COMMAND "${GIT_EXECUTABLE}" rev-parse --path-format=absolute
                --git-path HEAD --git-path packed-refs
            WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
            OUTPUT_VARIABLE _seds_git_paths OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
        execute_process(COMMAND "${GIT_EXECUTABLE}" symbolic-ref -q HEAD
            WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
            OUTPUT_VARIABLE _seds_git_ref OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
        if(_seds_git_ref)
            execute_process(COMMAND "${GIT_EXECUTABLE}" rev-parse --path-format=absolute
                    --git-path "${_seds_git_ref}"
                WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
                OUTPUT_VARIABLE _seds_ref_path OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
            string(APPEND _seds_git_paths "\n${_seds_ref_path}")
        endif()
        string(REPLACE "\n" ";" _seds_git_paths "${_seds_git_paths}")
        foreach(_seds_path IN LISTS _seds_git_paths)
            if(_seds_path AND EXISTS "${_seds_path}")
                set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${_seds_path}")
            endif()
        endforeach()
    endif()
endif()
message(STATUS "SD format firmware revision: ${SEDS_FIRMWARE_REVISION}")
# Only the SD writer needs it, so a new revision recompiles one file.
set_property(SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/sd_card.c" APPEND PROPERTY
    COMPILE_DEFINITIONS "SEDS_FIRMWARE_REVISION=\"${SEDS_FIRMWARE_REVISION}\"")
