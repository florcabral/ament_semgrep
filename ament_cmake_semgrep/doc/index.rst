ament_semgrep
==============

Performs a static code analysis of source files using the `Semgrep
<https://github.com/returntocorp/semgrep>`_ static analyzer engine.


How to run the check from the command line?
-------------------------------------------

The command line tool is provided by the package `ament_semgrep
<https://github.com/ament/ament_lint>`_.


How to run the check from within a CMake ament package as part of the tests?
----------------------------------------------------------------------------

``package.xml``:

.. code:: xml

    <buildtool_depend>ament_cmake</buildtool_depend>
    <test_depend>ament_cmake_semgrep</test_depend>

``CMakeLists.txt``:

.. code:: cmake

    find_package(ament_cmake REQUIRED)
    if(BUILD_TESTING)
      find_package(ament_cmake_semgrep REQUIRED)
      ament_semgrep()
    endif()

When running multiple linters as part of the CMake tests the documentation of
the package `ament_lint_auto <https://github.com/ament/ament_lint/tree/rolling/ament_lint_auto>`_ might
contain some useful information.

The documentation of the package `ament_cmake_test
<https://github.com/ament/ament_cmake/tree/rolling/ament_cmake_test>`_ provides more information on testing
in CMake ament packages.
