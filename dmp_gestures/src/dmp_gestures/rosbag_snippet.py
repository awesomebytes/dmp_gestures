#!/usr/bin/python

import rosbag
from std_msgs.msg import Int32, String

bag = rosbag.Bag('test.bag', 'w')

try:
    str = String()
    str.data = 'foo'

    i = Int32()
    i.data = 42

    bag.write('chatter', str)
    bag.write('numbers', i)
finally:
    bag.close()