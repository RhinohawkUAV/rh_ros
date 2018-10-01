#!/usr/bin/env python
#
# Upload a QGroundControl plan as a Rhinohawk mission
# 

import sys

from rh_autonomy.util import get_proxy
from rh_msgs.srv import SetMission
from rh_msgs.msg import Mission

import yaml

PUBLISH = False

mission_filepath = sys.argv[1]
print("Loading %s" % mission_filepath)

stream = open(mission_filepath, "r")
data = yaml.load(stream)

def load_obj(data, msg_class, level=0):

    ind = "  "*level
    obj = msg_class()
    print("")
    print(ind+"Loading %s"%msg_class)
    
    for field_name, field_type in zip(msg_class.__slots__,msg_class._slot_types):
   
        next_data = data[field_name]
        print(ind+"- parsing field %s (%s)" % (field_name,field_type))
    
        if "/" not in field_type:
            # simple attribute
            setattr(obj,field_name,next_data)
            print(ind+"    set %s = %s" % (field_name,next_data))
            continue

        pkg,cls = field_type.split("/")
        islist = False

        if cls.endswith("[]"):
            cls = cls[0:-2]
            islist = True

        fqcn = "%s.msg.%s" % (pkg,cls)

        print(ind+"  %s - %s - islist:%s"%(field_name,fqcn,islist))
        print(ind+"  %s"%next_data)

        module = __import__(pkg).msg # TODO: explore all submodules
        next_class =getattr(module, cls)

        if islist:
            values = []
            for next_data_elem in next_data:       
                value = load_obj(next_data_elem, next_class, level+1)
                values.append(value)
                
            setattr(obj,field_name,values)
            
        else:
            value = load_obj(next_data, next_class, level+1)
            setattr(obj,field_name,value)
            
    return obj

        
m = load_obj(data, Mission)
print(m)

if PUBLISH:
    set_mission = get_proxy('/rh/command/set_mission', SetMission)
    if set_mission(m):
        print("Successfully set mission")
        sys.exit(0)
    else:
        print("Problem setting mission")
        sys.exit(1)


