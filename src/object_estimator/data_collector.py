#!/usr/bin/env python
"""
MIT License

Copyright (c) 2018 Daehyung Park

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""
import rospy
from std_msgs.msg import String
import json
import datetime
import os, sys

import roslaunch
import rospkg

    
if __name__ == '__main__':
    """This is a sample code for repetitive data collection
    """
    rospy.init_node('data_collector')

    rospack = rospkg.RosPack()
    event_running= False
    cur_event    = None

    def seq_finished_callback(msg):
        global event_running
        event_running = False

    def event_callback(msg):
        global cur_event
        cur_event=msg.data

    rospy.sleep(1.0)

    # Communication 
    pub = rospy.Publisher('/symbol_grounding', String, queue_size=10)
    rospy.Subscriber('sequence_finished', String, seq_finished_callback)
    rospy.Subscriber('current_event', String, event_callback)
    rospy.sleep(1.0)

    path = os.path.join(rospack.get_path('object_estimator'),'launch/record_topic.launch')
    recording = False

    
    ## while not rospy.is_shutdown():

        ## cmd = input("Command? ")
        
        ## if event_running:
        ##     rospy.sleep(1.0)
        ##     continue

    # Publish a manipulation command 
    d = {"timestamp": str(datetime.datetime.now()),
         "type": "assemble",
         "params": {"1": {"primitive_action": "record",
                          "object": "cup1",
                          "source": "cup1",
                          "destination": "place-table2"}},
                          "param_num": 1 }            
    event_running = True
    pub.publish(json.dumps(d))


    # Run
    while not rospy.is_shutdown():
        if cur_event is None: continue
        if cur_event.find("z_approach")>=0 and recording is False:
            recording = True
            print "Start to record data"
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            launch = roslaunch.parent.ROSLaunchParent(uuid, [path])
            launch.start()
        elif cur_event.find("wing")>=0 and recording is True:
            recording = False
            print "Finish to record data"
            launch.shutdown()
            break
        rospy.sleep(0.1)

