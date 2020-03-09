#!/bin/env python
#
#  Copyright (c) 2020, Intel Corporation
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#      * Redistributions of source code must retain the above copyright
#        notice, this list of conditions and the following disclaimer.
#
#      * Redistributions in binary form must reproduce the above copyright
#        notice, this list of conditions and the following disclaimer in
#        the documentation and/or other materials provided with the
#        distribution.
#
#      * Neither the name of Intel Corporation nor the names of its
#        contributors may be used to endorse or promote products derived
#        from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
#  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY LOG OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

'''
Script for populating a PolicyStore database with default values for each agent.
'''

import sys
import geopmpy.agent
import geopmpy.policy_store

if len(sys.argv) != 2:
    sys.stderr.write("Provide path to PolicyStore to initialize.\n")
    sys.exit(1)

geopmpy.policy_store.connect(sys.argv[1])

# 1. Steps to populate DB with all-NAN policies (agent defaults; usually hardware maximum)
all_agents = geopmpy.agent.names()
for agent in all_agents:
    if len(geopmpy.agent.policy_names(agent)) > 0:
        geopmpy.policy_store.set_default(agent, [float('nan')])

# 2. Set specific defaults for each agent
geopmpy.policy_store.set_default("power_balancer", [180])

# 3. Set specific best policies for jobs with a profile name
geopmpy.policy_store.set_best("power_balancer", "myprofile", [200])

geopmpy.policy_store.disconnect()
