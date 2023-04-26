find_package(ament_cmake_test QUIET REQUIRED)

include("${ament_cmake_semgrep_DIR}/ament_semgrep.cmake")

ament_register_extension("ament_lint_auto" "ament_cmake_semgrep"
  "ament_cmake_semgrep_lint_hook.cmake")
