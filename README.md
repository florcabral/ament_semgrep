# ament_semgrep

Analyzes files using the [Semgrep](https://github.com/returntocorp/semgrep) static analysis engine.
See available rulesets in the [Semgrep Registry](https://semgrep.dev/r).

## Install

`cd ament_semgrep`

`python3 setup.py install`

## Quick test

`ament_semgrep`

Runs Semgrep recursively on the current directory. If run without specifying output file, prints the errors found and error count to the screen. 

### Examples

Get output file in `json` format:

`ament_semgrep -f=json -o=semgrep-results.json`

Get `xunit` formatted results:

`ament_semgrep --xunit-file xunit-results.xml`

Apply only [Python registry ruleset](https://semgrep.dev/p/python) to project:

`ament_semgrep -c=p/python`

## Running from the command line

The ament_semgrep tool has the following command line options:

    usage: ament_semgrep [-h] [-f FORMAT] [-c CONFIG] [-o OUTPUT] [-x [EXCLUDE [EXCLUDE ...]]] 
            [--xunit-file XUNIT_FILE] [--semgrep-version] [paths [paths ...]]

    Analyze source code using the Semgrep static analyzer.

    positional arguments:
      paths                 Files and/or directories to be checked. Directories are searched 
                            recursively for files ending in ending in '.c', '.cc', '.cpp', '.cxx', 
                            '.h', '.hh', '.hpp', '.hxx', '.py', '.sh', '.bash', '.xml', '.yaml', 
                            '.json'. (default: ['.'])

    optional arguments:
      -h, --help            Show this help message and exit
      -f --format           Specify output format 
                            {junit-xml,sarif,json,text,emacs,vim,gitlab-sast,gitlab-secrets}
      -c, --config          Configuration file or Semgrep Registry entry name.
                            (default: auto. Automatically fetch rules from the Registry 
                            tailored to your project)
      -o --output           Write report to filename
      -x [EXCLUDE [EXCLUDE ...]], --exclude [EXCLUDE [EXCLUDE ...]]
                            Exclude files from being checked. (default: None)
      --xunit-file XUNIT_FILE
                            Generate a xunit compliant XML file (default: None)
      --semgrep-version     Get the Semgrep version, print it, and then exit

## Adding ament_semgrep to unit tests

CMake integration is provided by the package [ament_cmake_semgrep](https://github.com/ament/ament_lint).
