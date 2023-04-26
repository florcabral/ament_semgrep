ament_semgrep
==============

Performs a static code analysis of source files using `Semgrep
<https://github.com/returntocorp/semgrep/>`_.
Files with the following extensions are being considered:
``.c``, ``.cc``, ``.cpp``, ``.cxx``, ``.h``, ``.hh``, ``.hpp``, ``.hxx``, ``.py``, ``.sh``, ``.bash``, ``.xml``, ``.yaml``, ``.json``.


How to run the check from the command line?
-------------------------------------------

.. code:: sh

    ament_semgrep [<path> ...]


How to run the check from within a CMake ament package as part of the tests?
----------------------------------------------------------------------------

The CMake integration is provided by the package `ament_cmake_semgrep
<https://github.com/ament/ament_lint>`_.
