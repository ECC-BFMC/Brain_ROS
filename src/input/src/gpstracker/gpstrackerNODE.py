#!/usr/bin/env python3

# Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE

import server_data
import server_listener
import server_subscriber
import position_listener

import time

import rospy

from utils.msg import localisation

class gpstrackerNODE():
    
    def __init__(self, ID):
        """ GpsTracker targets to connect on the server and to receive the messages, which incorporates 
        the coordinate of the robot on the race track. It has two main state, the setup state and the listening state. 
        In the setup state, it creates the connection with server. It's receiving  the messages from the server in the listening
        state. 

        It's a thread, so can be running parallel with other threads. You can access to the received parameters via 'coor' function.

        Examples
        --------
        Here you can find a simple example, where the GpsTracker are running 10 second:
            | gpstracker = GpsTracker()
            | gpstracker.start()
            | time.sleep(10)
            | gpstracker.stop()
            | gpstracker.join()

        """
        #: serverData object with server parameters
        self.__server_data = server_data.ServerData()
        #: discover the parameters of server
        self.__server_listener = server_listener.ServerListener(self.__server_data)
        #: connect to the server
        self.__subscriber = server_subscriber.ServerSubscriber(self.__server_data,ID)
        #: receive and decode the messages from the server
        self.__position_listener = position_listener.PositionListener(self.__server_data)
        
        rospy.init_node('gpstrackerNODE', anonymous=False)
        
        # BNO publisher object
        self.GPS_publisher = rospy.Publisher("/automobile/localisation", localisation, queue_size=1)
        
        
    #================================ RUN ========================================
    def run(self):
        rospy.loginfo("starting gpstrackerNODE")
        time.sleep(5)
        self._getting()
    
    #================================ GETTING ========================================
    def _getting(self):
        while not rospy.is_shutdown():
            try:
                self.setup()
                loc_data = self.listen()
                if loc_data is not None:
                    loc=localisation()
                    loc.timestamp = loc_data['timestamp']
                    loc.posA = loc_data['coor'][0]['real']
                    loc.posB = loc_data['coor'][0]['imag']
                    loc.rotA = loc_data['coor'][1]['real']
                    loc.rotB = loc_data['coor'][1]['imag']
                    
                    self.GPS_publisher.publish(loc)
            except Exception as e:
                pass
            

    def setup(self):
        """Actualize the server's data and create a new socket with it.
        """
        # discover the parameters of server
        if self.__server_data.socket == None:
            
            self.__server_listener.find()
            
            if self.__server_data.is_new_server:
                # connect to the server 
                self.__subscriber.subscribe()
        
    def listen(self):
        """ Listening the coordination of robot
        """
        return self.__position_listener.listen()

if __name__ == '__main__':
    gptrkNODE = gpstrackerNODE(4)
    gptrkNODE.run()
