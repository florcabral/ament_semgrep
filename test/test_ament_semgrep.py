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

from ament_semgrep.main import main


def test_exclude():
    """
    Checks that excluding a file relatively/absolutely works as expected.
    
    `ament_semgrep --exclude <filename>` should exclude <filename>.
    """
    rc = main(
        argv=[
            '--exclude',
            'test1.py'
        ])
    assert rc == 0


def test_xunit_file():
    """
    Checks that generating an XUnit XML file works as expected.
        
    `ament_semgrep --xunit-file <filename>` should generate XUnit file <filename>.
    """
    rc = main(
        argv=[
            '--xunit-file',
            'test-xunit.xml'
        ])
    assert rc == 0


def test_output():
    """
    Checks that including an output argument works as expected.
            
    `ament_semgrep --output <filename>` should generate file <filename>.
    """
    rc = main(
        argv=[
            '--output',
            'test-output.json'
        ])
    assert rc == 0


def test_format():
    """
    Checks that including a format argument works as expected.

    `ament_semgrep --format <format>` should generate file of format <format>.
    """
    rc = main(
        argv=[
            '--format',
            'json'
        ])
    assert rc == 0
    
