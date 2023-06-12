# Copyright 2023 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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
