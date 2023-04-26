#
# Add a test to perform static code analysis with Semgrep.
#
# :param TESTNAME: the name of the test, default: "semgrep"
# :type TESTNAME: string
# :param EXCLUDE: an optional list of exclude files or directories
# :type EXCLUDE: list
# :param CONFIG: output format
# :type CONFIG: string
# :param CONFIG: rules to apply in the scan
# :type CONFIG: string
# :param OUTPUT: filename of output report
# :type OUTPUT: string
# :param ARGN: the files or directories to check
# :type ARGN: list of strings
#
# @public
#
function(ament_semgrep)
  cmake_parse_arguments(ARG "" "LANGUAGE;TESTNAME" "EXCLUDE;FORMAT;CONFIG;OUTPUT" ${ARGN})
  if(NOT ARG_TESTNAME)
    set(ARG_TESTNAME "semgrep")
  endif()

  find_program(ament_semgrep_BIN NAMES "ament_semgrep")
  if(NOT ament_semgrep_BIN)
    message(FATAL_ERROR "ament_semgrep() could not find program 'ament_semgrep'")
  endif()

  set(result_file "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/${ARG_TESTNAME}.xunit.xml")
  set(cmd "${ament_semgrep_BIN}" "--xunit-file" "${result_file}")
  list(APPEND cmd ${ARG_UNPARSED_ARGUMENTS})
  if(ARG_EXCLUDE)
    list(APPEND cmd "--exclude" "${ARG_EXCLUDE}")
  endif()
  if(ARG_FORMAT)
    list(APPEND cmd "--format" "${ARG_FORMAT}")
  endif()
  if(ARG_CONFIG)
    list(APPEND cmd "--config" "${ARG_CONFIG}")
  endif()
  if(ARG_OUTPUT)
    list(APPEND cmd "--output" "${ARG_OUTPUT}")
  endif()

  file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/ament_semgrep")
  ament_add_test(
    "${ARG_TESTNAME}"
    COMMAND ${cmd}
    TIMEOUT 300
    OUTPUT_FILE "${CMAKE_BINARY_DIR}/ament_semgrep/${ARG_TESTNAME}.txt"
    RESULT_FILE "${result_file}"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  )
  set_tests_properties(
    "${ARG_TESTNAME}"
    PROPERTIES
    LABELS "semgrep;linter"
  )
endfunction()
