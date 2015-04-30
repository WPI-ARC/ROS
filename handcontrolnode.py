#!/usr/bin/env python

import rospy
from mqp.msg import *

import sys, termios, tty
from select import select

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
        self.doprint = True
        self.mode = 1

        self.values = [[1,    -1],
                       [0.01, -0.01],
                       [0.01, -0.01],
                       [0.01, -0.01],
                       [0.01, -0.01]]

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
Press x to quit
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
Press x to quit
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
Press x to quit
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
Press x to quit
"""

        #I know totally clunky here, but whatever... copy pasta is easy
        self.settingBindings = {
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
                'q':(0,0), #finger A control
                'a':(0,1),
                'w':(1,0), #finger B control
                's':(1,1),
                'e':(2,0), #finger C control
                'd':(2,1),
                'r':(3,0), #finger D control
                'f':(3,1),
                   }

        self.knuckleBindings= {
                'y':(0,0), #finger A 
                'h':(0,1),
                'u':(1,0), #finger B
                'j':(1,1),
                'i':(2,0), #finger C
                'k':(2,1),
                'o':(3,0), #finger D
                'l':(3,1),
                   }

    def getKey(self, timeout=0.1):
        # If this is being piped to, ignore non-blocking functionality
        if not sys.stdin.isatty():
            return sys.stdin.read(1)
        fileno = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fileno)
        ch = ''
        try:
            tty.setraw(fileno)
            rlist = [fileno]
            if timeout >= 0:
                [rlist, _, _] = select(rlist, [], [], timeout)
            if fileno in rlist:
                ch = sys.stdin.read(1)
        except Exception as ex:
            print "getch", ex
            raise OSError
        finally:
            termios.tcsetattr(fileno, termios.TCSADRAIN, old_settings)
        return ch

    def set_finger(self, finger, direction, mode):
        if(mode == 0):
            self.command.fingers[finger].jointangle += self.values[mode][direction]
        elif(self.mode == 1):
            self.command.fingers[finger].dutycycle += self.values[mode][direction]
        elif(self.mode == 2):
            self.command.fingers[finger].force += self.values[mode][direction]
        elif(self.mode == 3):
            self.command.fingers[finger].position += self.values[mode][direction]
        elif(self.mode == 4):
            self.command.fingers[finger].pressure += self.values[mode][direction]       

    def print_msg(self, val):
        if(val == 1):
            print self.duty_msg
        elif(val == 2):
            print self.force_msg
        elif(val == 3):
            print self.pos_msg
        elif(val == 4):
            print self.pres_msg        

    #emergency stop function
    def resetAll(self):
        self.mode = 1
        self.doprint = True
        for i in [0,1,2,3]:
            self.command.fingers[i].controlmode = 1
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
        # print "Received:", repr(key)

        if key in self.settingBindings.keys():
            self.mode = self.settingBindings[key]
            for i in [0,1,2,3]:
                self.command.fingers[i].controlmode = self.mode
            self.pub.publish(self.command)
            self.doprint = True

        elif key in self.controlBindings.keys():
            self.set_finger(self.controlBindings[key][0], self.controlBindings[key][1], self.mode)
            self.pub.publish(self.command)

        elif key in self.knuckleBindings.keys():
            self.set_finger(self.knuckleBindings[key][0], self.knuckleBindings[key][1], 0)
            self.pub.publish(self.command)
   
        elif (key == 't') or (key == 'g') or (key == ' '):
            self.resetAll()
            self.pub.publish(self.command)
            self.doprint = True

        elif key == 'x':
            print "shutting down"
            rospy.signal_shutdown("Keyboard interrupt")

if __name__ == "__main__":
    controlnode = HandControlNode() 
    while not rospy.is_shutdown():
        controlnode.tick()
