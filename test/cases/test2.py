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

import os as o
import subprocess as subp

# Vulnerable to wildcard injection
o.system("/bin/tar xvzf *")
o.system('/bin/chown *')
o.popen2('/bin/chmod *')
subp.Popen('/bin/chown *', shell=True)

# Not vulnerable to wildcard injection
subp.Popen('/bin/rsync *')
subp.Popen("/bin/chmod *")
subp.Popen(['/bin/chown', '*'])
subp.Popen(["/bin/chmod", sys.argv[1], "*"],
                 stdin=subprocess.PIPE, stdout=subprocess.PIPE)
o.spawnvp(os.P_WAIT, 'tar', ['tar', 'xvzf', '*'])
