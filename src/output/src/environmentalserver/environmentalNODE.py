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
import environmental_streamer

import time
import random

import rospy

from utils.msg import environmental

class environmentalNODE():
    
    def __init__(self, ID = 120):
        """ EnvironmentalHandler targets to connect on the server and to send messages, which incorporates 
        the coordinate of the encountered obstacles on the race track. It has two main state, the setup state and the streaming state. 
        In the setup state, it creates the connection with server. It's sending the messages to the server in the streaming
        state. 

        It's a thread, so can be run parallel with other threads. You can write the coordinates and the id of the encountered obstacle 
        and the script will send it.

        """
        #: serverData object with server parameters
        self.__server_data = server_data.ServerData()
        #: discover the parameters of server
        self.__server_listener = server_listener.ServerListener(self.__server_data)
        #: connect to the server
        self.__subscriber = server_subscriber.ServerSubscriber(self.__server_data,ID)
        #: receive and decode the messages from the server
        self.__environmental_streamer = environmental_streamer.EnvironmentalStreamer(self.__server_data)
        
        rospy.init_node('environmentalNODE', anonymous=False)
        
        # Environmental subscriber object
        self.ENVIRONMENTAL_subscriber = rospy.Subscriber("/automobile/environment", environmental, self._send)
        
        
    # ===================================== RUN ==========================================
    def run(self):
        """Apply the initializing methods and start subscriber.
        """
        rospy.loginfo("starting environmentalNODE")
        
        rospy.spin()

    def setup(self):
        """Actualize the server's data and create a new socket with it.
        """
        # Running while it has a valid connection with the server
        if (self.__server_data.socket == None):
            # discover the parameters of server
            self.__server_listener.find()
            if self.__server_data.is_new_server:
                # connect to the server 
                self.__subscriber.subscribe()
        

    def _send(self, msg):
        try:
            self.__environmental_streamer.sent = False
            while self.__environmental_streamer.sent == False and not rospy.is_shutdown():
                self.setup()
                self.__environmental_streamer.stream(msg.obstacle_id, msg.x, msg.y)
                print(msg)
        except:
            pass

if __name__ == '__main__':
    envNOD = environmentalNODE()
    envNOD.run()
