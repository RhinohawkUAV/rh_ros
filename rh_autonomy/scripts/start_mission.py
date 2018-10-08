#!/usr/bin/env python
#
# Start the Rhinohawk mission
# 

import sys

from rh_autonomy.util import get_proxy
from rh_msgs.srv import StartMission

start_mission = get_proxy('/rh/command/start_mission', StartMission)
res = start_mission()
if res and res.success:
    print("Successfully started mission")
    sys.exit(0)
else:
    print("Problem starting mission")
    sys.exit(1)

