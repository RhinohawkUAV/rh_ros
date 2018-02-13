#
# Utility classes for latching changing topic values.
#

import rospy


class LatchedValue:
    def __init__(self, topic, max_age, value):
        self.topic = topic
        self.max_age = max_age
        self.value = value
        self.time_acquired = rospy.get_rostime()


class LatchMap:

    def __init__(self):
        self.values = {}

    def latch_value(self, topic, max_age, value):
        self.values[topic] = LatchedValue(topic, max_age, value)


    def get_value(self, topic):
        if topic in self.values:
            lvalue = self.values[topic]
            d = lvalue.time_acquired - rospy.Time.now()
            if d.secs > lvalue.max_age:
                rospy.loginfo("Latched value too old (%d secs)", d.secs)
                return None
            return lvalue.value
        return None



