find_package(ament_cmake_core REQUIRED)

file(GLOB_RECURSE _source_files FOLLOW_SYMLINKS
  "*.c"
  "*.cc"
  "*.cpp"
  "*.cxx"
  "*.h"
  "*.hh"
  "*.hpp"
  "*.hxx"
  "*.py"
  "*.sh"
  "*.bash"
  "*.xml"  
  "*.yaml"
  "*.json"
)
if(_source_files)
  message(STATUS "Added test 'semgrep' to perform static analysis")

  # Get exclude paths for added targets
  set(_all_exclude "")
  if(DEFINED ament_cmake_semgrep_ADDITIONAL_EXCLUDE)
    list(APPEND _all_exclude ${ament_cmake_semgrep_ADDITIONAL_EXCLUDE})
  endif()

  if(DEFINED AMENT_LINT_AUTO_FILE_EXCLUDE)
    list(APPEND _all_exclude ${AMENT_LINT_AUTO_FILE_EXCLUDE})
  endif()

  message(
    STATUS "Configured semgrep exclude dirs and/or files: ${_all_exclude}"
  )
  ament_semgrep(
    EXCLUDE ${_all_exclude}
  )
endif()
