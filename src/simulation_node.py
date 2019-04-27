#!/usr/bin/env python
# Copyright 2019 Philip Hopley
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not 
# use this file except in compliance with the License. You may obtain a  copy
# of the License at
#
#  http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software 
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.
#
# Simulation test code
import sys
import rospy
from pi_io.srv import gpio_output
from std_srvs.srv import Empty
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

class SimNode:
    def __init__(self):
        # service that will expect to be running        
        self.__gpio_service = rospy.Service('gpio/output_cmd', gpio_output, self.OutputCommand) 
        self.__stop_motor_service = rospy.Service('stop_motor', Empty, self.StopCommand)
        self.__start_motor_service = rospy.Service('start_motor', Empty, self.StartCommand)
        
        # Accept a pan_tilt_node/joints message and turn it into a 
        # head_controller/command message so that the head moves in Gazebo
        self.__pan_tilt_sub = rospy.Subscriber('pan_tilt_node/joints', JointState, self.PanTiltCallback)
        self.__head_control_pub_ = rospy.Publisher('head_controller/command', JointTrajectory, queue_size=1)
        

    # Callback for head joints message
    def PanTiltCallback(self, msg):        
        
        jt = JointTrajectory()
     
        jt.joint_names = ["head_pan", "head_tilt"]
    
        point = JointTrajectoryPoint()
        
        # We could read the names of the joints but we know that pan is in [0] and tilt in [1]
        point.positions = [msg.position[0], msg.position[1]]
        point.time_from_start.secs = 1
        jt.points = [point]
                
        self.__head_control_pub_.publish(jt)
    
    def OutputCommand(self, request):
        # Just return True        
        return True
        
    def StopCommand(self, request):               
        return []
        
    def StartCommand(self, request):        
        return []              

                
def main(args):
    rospy.init_node('sim_node', anonymous=False)
    rospy.loginfo("sim node started")    
    sim = SimNode()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Shutting down')

if __name__ == '__main__':
    main(sys.argv)
