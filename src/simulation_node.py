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

class SimNode:
    def __init__(self):        
        self.__gpio_service = rospy.Service('gpio/output_cmd', gpio_output, self.OutputCommand) 
        self.__stop_motor_service = rospy.Service('stop_motor', Empty, self.StopCommand)
        self.__start_motor_service = rospy.Service('start_motor', Empty, self.StartCommand)

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
