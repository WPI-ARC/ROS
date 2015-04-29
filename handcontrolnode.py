#!/usr/bin/env python

import rospy
from mqp.msg import FingerSetpoint, HandSetpoint
import sys, select, tty

class HandControlNode:
    def __init__(self):
        rospy.init_node('control_by_hand') #hah get it??
        rospy.loginfo('Hand control node started')
        rospy.on_shutdown(self._on_shutdown)

        self._init_params()
        self.pub = rospy.Publisher('/softhand/setpoints', HandSetpoint, queue_size=1)

    def _on_shutdown(self):
        self.resetAll()
        self.pub.publish(self.command)

    def _init_params(self):
        self.command = HandSetpoint()
        self.command.fingers = [FingerSetpoint(), FingerSetpoint(), FingerSetpoint(), FingerSetpoint()]
        self.key = ''
        self.doprint = True
        self.mode = 0

        self.duty_msg = """
Finger Control Interface
---------------------------
You are controlling: DUTY CYCLE    | Set Finger Angles with:
   Finger A                        |    Finger A 
    + -> q                         |     + -> y
    - -> a                         |     - -> h      
                                   |
   Finger B                        |    Finger B 
    + -> w                         |     + -> u
    - -> s                         |     - -> j
                                   |
   Finger C                        |    Finger C 
    + -> e                         |     + -> i
    - -> d                         |     - -> k
                                   |
   Finger D                        |    Finger D 
    + -> r                         |     + -> o
    - -> f                         |     - -> l 
                                   |

Switch control methods using:
    2 - Force Control
    3 - Position Control
    4 - Pressure Control

Press t or g for emergency stop
Press CTRL-C to quit
"""

        self.force_msg = """
Finger Control Interface
---------------------------
You are controlling: FORCE OUTPUT  | Set Finger Angles with:
   Finger A                        |    Finger A 
    + -> q                         |     + -> y
    - -> a                         |     - -> h      
                                   |
   Finger B                        |    Finger B 
    + -> w                         |     + -> u
    - -> s                         |     - -> j
                                   |
   Finger C                        |    Finger C 
    + -> e                         |     + -> i
    - -> d                         |     - -> k
                                   |
   Finger D                        |    Finger D 
    + -> r                         |     + -> o
    - -> f                         |     - -> l 
                                   |

Switch control methods using:
    1 - Duty Cycle Control
    3 - Position Control
    4 - Pressure Control

Press t or g for emergency stop
Press CTRL-C to quit
"""

        self.pos_msg = """
Finger Control Interface
---------------------------
You are controlling: POSITION OUT  | Set Finger Angles with:
   Finger A                        |    Finger A 
    + -> q                         |     + -> y
    - -> a                         |     - -> h      
                                   |
   Finger B                        |    Finger B 
    + -> w                         |     + -> u
    - -> s                         |     - -> j
                                   |
   Finger C                        |    Finger C 
    + -> e                         |     + -> i
    - -> d                         |     - -> k
                                   |
   Finger D                        |    Finger D 
    + -> r                         |     + -> o
    - -> f                         |     - -> l 
                                   |

Switch control methods using:
    1 - Duty Cycle Control
    2 - Force Control
    4 - Pressure Control

Press t or g for emergency stop
Press CTRL-C to quit
"""

        self.pres_msg = """
Finger Control Interface
---------------------------
You are controlling: PRESSURE OUT  | Set Finger Angles with:
   Finger A                        |    Finger A 
    + -> q                         |     + -> y
    - -> a                         |     - -> h      
                                   |
   Finger B                        |    Finger B 
    + -> w                         |     + -> u
    - -> s                         |     - -> j
                                   |
   Finger C                        |    Finger C 
    + -> e                         |     + -> i
    - -> d                         |     - -> k
                                   |
   Finger D                        |    Finger D 
    + -> r                         |     + -> o
    - -> f                         |     - -> l 
                                   |

Switch control methods using:
    1 - Duty Cycle Control
    2 - Force Control
    3 - Position Control

Press t or g for emergency stop
Press CTRL-C to quit
"""

        #I know totally clunky here, but whatever... copy pasta is easy
        self.settingBindings = {
        '0':(0),
        '1':(1),
        '2':(2),
        '3':(3),
        '4':(4),
           }

        #the way it works is this: (index, val)
        #index - the index val of hand.fingers that we are accessing
        #val - what is added to existing val
        #btw setting val is added to is set with another var
        self.controlBindings = {
                'q':(0,1), #finger A control
                'a':(0,-1),
                'w':(1,1), #finger B control
                's':(1,-1),
                'e':(2,1), #finger C control
                'd':(2,-1),
                'r':(3,1), #finger D control
                'f':(3,-1),
                   }

        self.knuckleBindings= {
                'y':(0,0), #finger A 
                'h':(0,-1),
                'u':(1,1), #finger B
                'j':(1,-1),
                'i':(2,1), #finger C
                'k':(2,-1),
                'o':(3,1), #finger D
                'l':(3,-1),
                   }

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            self.key = str(sys.stdin.read(1))
        else:
            self.key = ''

    def set_finger(self, finger, direction):
        if(mode == 0):
            self.command.fingers[finger].dutycycle += self.values[self.mode][direction]
        elif(mode == 1):
            self.command.fingers[finger].force += self.values[self.mode][direction]
        elif(mode == 2):
            self.command.fingers[finger].position += self.values[self.mode][direction]
        elif(mode == 3):
            self.command.fingers[finger].pressure += self.values[self.mode][direction]
        elif(mode == 4):
            self.command.fingers[finger].jointangle += self.values[self.mode][direction]       

    def print_msg(self, val):
        if(val == 0):
            print self.duty_msg
        elif(val == 1):
            print self.force_msg
        elif(val == 2):
            print self.pos_msg
        elif(val == 3):
            print self.pres_msg        

    #emergency stop function
    def resetAll(self):
        for i in [0,1,2,3]:
            self.command.fingers[i].force = 0
            self.command.fingers[i].position = 0
            self.command.fingers[i].pressure = 0
            self.command.fingers[i].dutycycle = 0
            self.command.fingers[i].jointangle = 0

    def tick(self):
        # print "Tick"
        if(self.doprint):
            self.print_msg(self.mode)
            self.doprint = False

        key = self.getKey()

        #for button debugging
        print "Received:", self.key, "\r"

        if key in self.settingBindings.keys():
            self.mode = self.settingBindings[key]
            self.pub.publish(self.command)
            self.doprint = True

        elif key in self.controlBindings.keys():
            self.set_finger(hand, self.controlBindings[key])
            self.pub.publish(self.command)

        elif key in self.knuckleBindings.keys():
            self.set_finger(hand, self.controlBindings[key], 5)
            self.pub.publish(self.command)

        #keys t, g, or 'space' command estop    
        elif key == 't' or key == 'g' or key == ' ':
            self.resetAll()
            self.pub.publish(self.command)
            self.doprint = True

        elif key == 'x':
            rospy.signal_shutdown("Keyboard interrupt")

if __name__ == "__main__":
    controlnode = HandControlNode() 
    while not rospy.is_shutdown():
        controlnode.tick()
        # rospy.spin()
