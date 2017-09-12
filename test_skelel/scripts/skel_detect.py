#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic
import rospy
from std_msgs.msg import String
import qi
import time
import sys
import argparse

class HumanGreeter(object):
    """
    A simple class to react to face detection events.
    """

    def __init__(self, app):
        #ROS
        self.pub = rospy.Publisher('chatter', String, queue_size=10)
        rospy.init_node('talker', anonymous=True)
        self.rate = rospy.Rate(30)
        print "Initilization"
        """
        Initialisation of qi framework and event detection.
        """
        super(HumanGreeter, self).__init__()
        app.start()
        session = app.session
        # Get the service ALMemory.
        self.memory = session.service("ALMemory")
        # Connect the event callback.
        self.subscriber = self.memory.subscriber("SkeletonDetector/jointsData")
        self.subscriber.signal.connect(self.on_human_tracked)
        # Get the services ALTextToSpeech and ALFaceDetection.
        # self.tts = session.service("ALTextToSpeech")
        self.skeleton_detector = session.service("ALSkeletonDetector")
        self.skeleton_detector.subscribe("HumanGreeter")
        self.got_face = False
        # preparing string of Skeleton value
        self.value_start = "start"
        self.value_end   = "end"
    def on_human_tracked(self, value):
        """
        Callback for event FaceDetected.
        """
        # qi.info("on_human_tracked","okey")
        if type(value[0]) is str:
            rospy.loginfo("No Skeleton")
            # qi.info("skel",str(value))
            skeleton = [self.value_start,0,self.value_end]
            self.pub.publish(str(skeleton))
        else:

            # value[0][0] = "head"
            # value[1][0] = "left_hand"
            # value[2][0] = "right_hand"
            # value[3][0] = "left_shoulder"
            # value[4][0] = "right_shoulder"
            # value[5][0] = "left_elbow"
            # value[6][0] = "right_elbow"
            # qi.info("skel",value[0][0] + "," + str(float(value[0][1])) + "\t" + str(float(value[0][2])))
            qi.info("skel",value[0][0] + "\t" + str(float(value[0][1])) + "\t" + str(float(value[0][2])) + "\t" + str(float(value[0][3])) )
            skeleton =[self.value_start,value[0][0],float(value[0][1]),float(value[0][2]),float(value[0][3]),
                                        value[1][0],float(value[1][1]),float(value[1][2]),float(value[1][3]),
                                        value[2][0],float(value[2][1]),float(value[2][2]),float(value[2][3]),
                                        value[3][0],float(value[3][1]),float(value[3][2]),float(value[3][3]),
                                        value[4][0],float(value[4][1]),float(value[4][2]),float(value[4][3]),
                                        value[5][0],float(value[5][1]),float(value[5][2]),float(value[5][3]),
                                        value[6][0],float(value[6][1]),float(value[6][2]),float(value[6][3]),self.value_end]
        self.pub.publish(str(skeleton))
        # self.rate.sleep()
    def run(self):
        """
        Loop on, wait for events until manual interruption.
        """
        print "Starting HumanGreeter"
        try:
            while not rospy.is_shutdown():
                self.rate.sleep()
        except rospy.ROSInterruptException:
            print "Interrupted by user, stopping HumanGreeter"
            self.skeleton_detector.unsubscribe("HumanGreeter")
            sys.exit(0)
            pass


if __name__ == "__main__":
    try:
        ip = "150.65.205.92"
        port = 9559
        connection_url = "tcp://" + ip + ":" + str(port)
        app = qi.Application(["HumanGreeter", "--qi-url=" + connection_url])
    except RuntimeError:
        print ("Can't connect to Naoqi at ip \"" + args.ip + "\" on port " + str(args.port) +".\n"
               "Please check your script arguments. Run with -h option for help.")
        sys.exit(1)

    human_greeter = HumanGreeter(app)
    human_greeter.run()


