#!/usr/bin/python

"""
Class for sending keyboard commands to spot micro walk node, control velocity, yaw rate, and walk event 
"""
import rospy
import sys, select, termios, tty, time # For terminal keyboard key press reading
from std_msgs.msg import Float32, Bool 
from geometry_msgs.msg import Vector3
from math import pi
from ASRController import ASR
from threading import Thread
from Gamepad import Gamepad

msg = """
Spot Micro Walk Command

Enter one of the following options:
-----------------------------
quit: stop and quit the program
walk: Start walk mode and keyboard motion control
stand: Stand robot up

Keyboard commands for body motion 
---------------------------
   q   w   e            u
   a   s   d    
            

  u: Quit body motion command mode and go back to rest mode
  w: Increment forward speed command / decrease pitch angle
  a: Increment left speed command / left roll angle
  s: Increment backward speed command / increase pitch angle
  d: Increment right speed command / right roll angle
  q: Increment body yaw rate command / left yaw angle (negative left, positive right) 
  e: Increment body yaw rate command / right yaw angle (negative left, positive right) 
  f: In walk mode, zero out all rate commands.

  anything else : Prompt again for command


CTRL-C to quit
"""
valid_cmds = ('quit','Quit','walk','stand','idle', 'angle_cmd')

# Global body motion increment values
speed_inc = 0.02
yaw_rate_inc = 2*pi/180
angle_inc = 2.5*pi/180

class GlobalState():
    def __init__(self, control):
        self.mode = 'idle'
        self.control = control
    
    def change_mode(self, new_mode):
        if self.mode == new_mode: return

        if self.mode == 'walk' and self.mode != new_mode:
            self.control.ros_pub_state_cmd.publish(self.control._walk_event_cmd_msg)
            self.control.ros_pub_stand_cmd.publish(self.control._stand_event_cmd_msg)
            time.sleep(2)
        
        if new_mode == 'stand':
            self.control.ros_pub_stand_cmd.publish(self.control._stand_event_cmd_msg)
        
        if new_mode == 'idle':
            self.control.ros_pub_idle_cmd.publish(self.control._idle_event_cmd_msg)
        
        if new_mode == 'walk':
            if self.mode == 'idle':
                self.control.ros_pub_stand_cmd.publish(self.control._stand_event_cmd_msg)
                time.sleep(2)
            
            self.control.reset_all_motion_commands_to_zero()

            # Publish walk event
            self.control.ros_pub_state_cmd.publish(self.control._walk_event_cmd_msg)
            self.control.ros_pub_walk_cmd.publish(self.control._walk_event_cmd_msg)
        
        if new_mode == 'angle':
            self.control.reset_all_angle_commands_to_zero()
        
        self.mode = new_mode

voice_input_state = False

def listen_asr():
    global state, voice_input_state

    asr = ASR(0x79, 3)
    asr.setMode(2)
    asr.eraseWords()
    asr.addWords(1, 'wang zai')

    # stand
    asr.addWords(2, 'zhan qi lai')
    asr.addWords(2, 'zhan li')

    # angle
    asr.addWords(3, 'tiao zheng jiao du')

    # walk
    asr.addWords(4, 'xing zou')

    # idle
    asr.addWords(5, 'tang xia')
    asr.addWords(5, 'xiu xi')

    # stop (wait)
    asr.addWords(6, 'ting xia')

    # walk forward
    asr.addWords(7, 'qian jin')
    asr.addWords(7, 'xiang qian zou')
    
    # look up
    asr.addWords(8, 'tai tou')

    # walk backward
    asr.addWords(9, 'hou tui')
    asr.addWords(9, 'xiang hou zou')
    
    # look down
    asr.addWords(10, 'di tou')

    # walk left
    asr.addWords(11, 'zuo yi')

    # lean left
    asr.addWords(12, 'zuo qing xie')

    # walk right
    asr.addWords(13, 'you yi')
    
    # lean right
    asr.addWords(14, 'you qing xie')

    # turn left
    asr.addWords(15, 'zuo zhuan wan')
    
    # turn right
    asr.addWords(16, 'you zhuan wan')

    # stand still
    asr.addWords(17, 'yuan di ta bu')

    invalid_count = 1

    while True:
        command = asr.getResult()

        if invalid_count > 0:
            invalid_count -= 1
            time.sleep(0.5)
            continue

        if not voice_input_state or command == 0:
            time.sleep(0.5)
            continue

        if command == 2:
            print('[ASR] stand')
            state.change_mode('stand')
        if command == 3:
            print('[ASR] angle')
            state.change_mode('angle')
        if command == 4:
            print('[ASR] walk')
            state.change_mode('walk')
        if command == 5:
            print('[ASR] idle')
            state.change_mode('idle')
        if command == 6:
            print('[ASR] wait')
            state.change_mode('wait')
        if command == 7:
            print('[ASR] walk forward')
            if state.mode == 'walk':
                state.control.walk_forward(rate=3)
        if command == 9:
            print('[ASR] walk backward')
            if state.mode == 'walk':
                state.control.walk_backward(rate=3)
        if command == 11:
            print('[ASR] walk left')
            if state.mode == 'walk':
                state.control.walk_left(rate=3)
        if command == 13:
            print('[ASR] walk right')
            if state.mode == 'walk':
                state.control.walk_right(rate=3)
        if command == 15:
            print('[ASR] turn left')
            if state.mode == 'walk':
                state.control.turn_left(rate=3)
        if command == 16:
            print('[ASR] turn right')
            if state.mode == 'walk':
                state.control.turn_right(rate=3)
        if command == 17:
            print('[ASR] stand still')
            if state.mode == 'walk':
                state.control.stand_still()
        time.sleep(1)


def listen_xbox():
    global state, voice_input_state
    while True:
        while not Gamepad.available():
            time.sleep(1)
        xbox = Gamepad.XboxOne()
        print('Xbox One Controller connected')
        
        while xbox.isConnected():
            event_type, control, value = xbox.getNextEvent()
            if control not in ['LEFT-X', 'LEFT-Y', 'RIGHT-X', 'RIGHT-Y', 'RT', 'LT', 'DX', 'DY', 'A', 'B', 'X', 'Y', 'LB', 'RB', 'MODE', 'LA', 'RA']:
                time.sleep(0.1)
                continue
            if event_type == 'BUTTON' and value:
                if control == 'X':
                    print('[XBOX] stand')
                    state.change_mode('stand')
                if control == 'Y':
                    print('[XBOX] angle')
                    state.change_mode('angle')
                if control == 'B':
                    print('[XBOX] walk')
                    if state.mode == 'walk':
                        state.control.stand_still()
                    else:
                        state.change_mode('walk')
                if control == 'A':
                    print('[XBOX] idle')
                    state.change_mode('idle')
                if control == 'LB':
                    print('[XBOX] turn left / rotate left')
                    if state.mode == 'walk':
                        state.control.turn_left(rate=2)
                    elif state.mode == 'angle':
                        state.control.rotate_left(rate=2)
                if control == 'RB':
                    print('[XBOX] turn right / rotate right')
                    if state.mode == 'walk':
                        state.control.turn_right(rate=2)
                    elif state.mode == 'angle':
                        state.control.rotate_right(rate=2)
                if control == 'MODE':
                    voice_input_state = not voice_input_state
                    print('[XBOX] voice control', 'ON' if voice_input_state else 'OFF')

            elif event_type == 'AXIS':
                if control == 'DY' and value < -0.5:
                    print('[XBOX] walk forward / look up')
                    if state.mode == 'walk':
                        state.control.walk_forward(rate=2)
                    elif state.mode == 'angle':
                        state.control.look_up(rate=2)
                if control == 'DY' and value > 0.5:
                    print('[XBOX] walk backward / look down')
                    if state.mode == 'walk':
                        state.control.walk_backward(rate=2)
                    elif state.mode == 'angle':
                        state.control.look_down(rate=2)
                if control == 'DX' and value < -0.5:
                    print('[XBOX] walk left / lean left')
                    if state.mode == 'walk':
                        state.control.walk_left(rate=2)
                    elif state.mode == 'angle':
                        state.control.lean_left(rate=2)
                if control == 'DX' and value > 0.5:
                    print('[XBOX] walk right / lean right')
                    if state.mode == 'walk':
                        state.control.walk_right(rate=2)
                    elif state.mode == 'angle':
                        state.control.lean_right(rate=2)
        
        print('Xbox One Controller disconnected')


class SpotMicroKeyboardControl():
    def __init__(self):

        # Create messages for body motion commands, and initialize to zero 
        self._x_speed_cmd_msg = Float32()
        self._x_speed_cmd_msg.data = 0

        self._y_speed_cmd_msg = Float32()
        self._y_speed_cmd_msg.data = 0

        self._yaw_rate_cmd_msg = Float32()
        self._yaw_rate_cmd_msg.data = 0

        self._angle_cmd_msg = Vector3()
        self._angle_cmd_msg.x = 0
        self._angle_cmd_msg.y = 0
        self._angle_cmd_msg.z = 0

        self._speed_cmd_msg = Vector3()
        self._speed_cmd_msg.x = 0
        self._speed_cmd_msg.y = 0
        self._speed_cmd_msg.z = 0

        self._walk_event_cmd_msg = Bool()
        self._walk_event_cmd_msg.data = True # Mostly acts as an event driven action on receipt of a true message

        self._stand_event_cmd_msg = Bool()
        self._stand_event_cmd_msg.data = True
       
        self._idle_event_cmd_msg = Bool()
        self._idle_event_cmd_msg.data = True

        rospy.loginfo("Setting Up the Spot Micro Keyboard Control Node...")

        # Set up and title the ros node for this code
        rospy.init_node('spot_micro_keyboard_control')

        # Create publishers for x,y speed command, body rate command, and state command
        self.ros_pub_x_speed_cmd    = rospy.Publisher('x_speed_cmd',Float32,queue_size=1)
        self.ros_pub_y_speed_cmd    = rospy.Publisher('/y_speed_cmd',Float32,queue_size=1)
        self.ros_pub_yaw_rate_cmd   = rospy.Publisher('/yaw_rate_cmd',Float32,queue_size=1)
        self.ros_pub_speed_cmd      = rospy.Publisher('/speed_cmd',Vector3,queue_size=1)
        self.ros_pub_angle_cmd      = rospy.Publisher('/angle_cmd',Vector3,queue_size=1)
        self.ros_pub_state_cmd      = rospy.Publisher('/state_cmd',Bool,queue_size=1)
        self.ros_pub_walk_cmd       = rospy.Publisher('/walk_cmd',Bool, queue_size=1)
        self.ros_pub_stand_cmd      = rospy.Publisher('/stand_cmd',Bool,queue_size=1)
        self.ros_pub_idle_cmd       = rospy.Publisher('/idle_cmd',Bool,queue_size=1)

        rospy.loginfo("> Publishers corrrectly initialized")

        rospy.loginfo("Initialization complete")

        # Setup terminal input reading, taken from teleop_twist_keyboard
        self.settings = termios.tcgetattr(sys.stdin)

    def reset_all_motion_commands_to_zero(self):
        '''Reset body motion cmd states to zero and publish zero value body motion commands'''
        self._x_speed_cmd_msg.data = 0
        self._y_speed_cmd_msg.data = 0
        self._yaw_rate_cmd_msg.data = 0

        self.ros_pub_x_speed_cmd.publish(self._x_speed_cmd_msg)
        self.ros_pub_y_speed_cmd.publish(self._y_speed_cmd_msg)
        self.ros_pub_yaw_rate_cmd.publish(self._yaw_rate_cmd_msg)

        self._speed_cmd_msg.x = 0
        self._speed_cmd_msg.y = 0
        self._speed_cmd_msg.z = 0

        self.ros_pub_speed_cmd.publish(self._speed_cmd_msg)

    def reset_all_angle_commands_to_zero(self):
        '''Reset angle cmd states to zero and publish them'''

        self._angle_cmd_msg.x = 0
        self._speed_cmd_msg.y = 0
        self._speed_cmd_msg.z = 0

        self.ros_pub_angle_cmd.publish(self._angle_cmd_msg)

    def walk_forward(self, rate=1):
        self._x_speed_cmd_msg.data = self._x_speed_cmd_msg.data + speed_inc * rate
        self.ros_pub_x_speed_cmd.publish(self._x_speed_cmd_msg)

        self._speed_cmd_msg.x = self._speed_cmd_msg.x + speed_inc * rate
        self.ros_pub_speed_cmd.publish(self._speed_cmd_msg)
    
    def walk_backward(self, rate=1):
        self._x_speed_cmd_msg.data = self._x_speed_cmd_msg.data - speed_inc * rate
        self.ros_pub_x_speed_cmd.publish(self._x_speed_cmd_msg)

        self._speed_cmd_msg.x = self._speed_cmd_msg.x - speed_inc * rate
        self.ros_pub_speed_cmd.publish(self._speed_cmd_msg)

    def walk_left(self, rate=1):
        self._y_speed_cmd_msg.data = self._y_speed_cmd_msg.data + speed_inc * rate
        self.ros_pub_y_speed_cmd.publish(self._y_speed_cmd_msg)

        self._speed_cmd_msg.y = self._speed_cmd_msg.y - speed_inc * rate
        self.ros_pub_speed_cmd.publish(self._speed_cmd_msg)

    def walk_right(self, rate=1):
        self._y_speed_cmd_msg.data = self._y_speed_cmd_msg.data - speed_inc * rate
        self.ros_pub_y_speed_cmd.publish(self._y_speed_cmd_msg)

        self._speed_cmd_msg.y = self._speed_cmd_msg.y + speed_inc * rate
        self.ros_pub_speed_cmd.publish(self._speed_cmd_msg)

    def turn_left(self, rate=1):
        self._yaw_rate_cmd_msg.data = self._yaw_rate_cmd_msg.data + yaw_rate_inc * rate
        self.ros_pub_yaw_rate_cmd.publish(self._yaw_rate_cmd_msg)

        self._speed_cmd_msg.z = self._speed_cmd_msg.z - yaw_rate_inc * rate
        self.ros_pub_speed_cmd.publish(self._speed_cmd_msg)

    def turn_right(self, rate=1):
        self._yaw_rate_cmd_msg.data = self._yaw_rate_cmd_msg.data - yaw_rate_inc * rate
        self.ros_pub_yaw_rate_cmd.publish(self._yaw_rate_cmd_msg)

        self._speed_cmd_msg.z = self._speed_cmd_msg.z + yaw_rate_inc * rate
        self.ros_pub_speed_cmd.publish(self._speed_cmd_msg)

    def stand_still(self):
        self._speed_cmd_msg.x = 0
        self._speed_cmd_msg.y = 0
        self._speed_cmd_msg.z = 0
        self._x_speed_cmd_msg.data = 0
        self._y_speed_cmd_msg.data = 0
        self._yaw_rate_cmd_msg.data = 0

        self.ros_pub_speed_cmd.publish(self._speed_cmd_msg)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        global state

        # Publish all body motion commands to 0
        self.reset_all_motion_commands_to_zero()

        # Prompt user with keyboard command information
        
        while not rospy.is_shutdown():
            print(msg)
            userInput = raw_input("Command?: ")

            if userInput not in valid_cmds:
                print('Valid command not entered, try again...')
            else:
                if userInput == 'quit':
                    print("Ending program...")
                    break
                
                elif userInput == 'stand':
                    state.change_mode('stand')
                    #Publish stand command event
                    #self.ros_pub_stand_cmd.publish(self._stand_event_cmd_msg)
                
                elif userInput == 'idle':
                    state.change_mode('idle')
                    #Publish idle command event
                    #self.ros_pub_idle_cmd.publish(self._idle_event_cmd_msg)

                elif userInput == 'angle_cmd':
                    state.change_mode('angle')
                    # Reset all angle commands
                    # self.reset_all_angle_commands_to_zero()
                    
                    # Enter loop to act on user command
                    print('Enter command, u to go back to command select: ')

                    while (1):
                        print('Cmd Values: phi: %1.3f deg, theta: %1.3f deg, psi: %1.3f deg '\
                                %(self._angle_cmd_msg.x*180/pi, self._angle_cmd_msg.y*180/pi, self._angle_cmd_msg.z*180/pi))
                       
                        userInput = self.getKey()

                        if userInput == 'u':
                            state.change_mode('wait')
                            # Break out of angle command mode 
                            break

                        elif userInput not in ('w','a','s','d','q','e','u'):
                            print('Key not in valid key commands, try again')
                        else:
                            if userInput == 'w':
                                self._angle_cmd_msg.y = self._angle_cmd_msg.y - angle_inc
                                self.ros_pub_angle_cmd.publish(self._angle_cmd_msg)
                            
                            elif userInput == 's':
                                self._angle_cmd_msg.y = self._angle_cmd_msg.y + angle_inc
                                self.ros_pub_angle_cmd.publish(self._angle_cmd_msg)

                            elif userInput == 'q':
                                self._angle_cmd_msg.z = self._angle_cmd_msg.z + angle_inc
                                self.ros_pub_angle_cmd.publish(self._angle_cmd_msg)

                            elif userInput == 'e':
                                self._angle_cmd_msg.z = self._angle_cmd_msg.z - angle_inc
                                self.ros_pub_angle_cmd.publish(self._angle_cmd_msg)

                            elif userInput == 'a':
                                self._angle_cmd_msg.x = self._angle_cmd_msg.x - angle_inc
                                self.ros_pub_angle_cmd.publish(self._angle_cmd_msg)

                            elif userInput == 'd':
                                self._angle_cmd_msg.x = self._angle_cmd_msg.x + angle_inc
                                self.ros_pub_angle_cmd.publish(self._angle_cmd_msg)

                elif userInput == 'walk':
                    state.change_mode('walk')
                    # Reset all body motion commands to zero
                    # self.reset_all_motion_commands_to_zero()

                    # Publish walk event
                    # self.ros_pub_state_cmd.publish(self._walk_event_cmd_msg)
                    # self.ros_pub_walk_cmd.publish(self._walk_event_cmd_msg)

                    # Enter loop to act on user command

                    print('Enter command, u to go back to stand mode: ')

                    while (1):
                        print('Cmd Values: x speed: %1.3f m/s, y speed: %1.3f m/s, yaw rate: %1.3f deg/s '\
                                %(self._x_speed_cmd_msg.data,self._y_speed_cmd_msg.data,self._yaw_rate_cmd_msg.data*180/pi))
                       
                        userInput = self.getKey()

                        if userInput == 'u':
                            state.change_mode('wait')
                            # Send walk event message, this will take robot back to rest mode
                            # self.ros_pub_state_cmd.publish(self._walk_event_cmd_msg)
                            # self.ros_pub_stand_cmd.publish(self._stand_event_cmd_msg)
                            break

                        elif userInput not in ('w','a','s','d','q','e','u','f'):
                            print('Key not in valid key commands, try again')
                        else:
                            if userInput == 'w':
                                self.walk_forward()
                            
                            elif userInput == 's':
                                self.walk_backward()

                            elif userInput == 'a':
                                self.walk_left()
                            
                            elif userInput == 'd':
                                self.walk_right()

                            elif userInput == 'q':
                                self.turn_left()

                            elif userInput == 'e':
                                self.turn_right()

                            elif userInput == 'f':
                                self.stand_still()

                                

if __name__ == "__main__":
    smkc     = SpotMicroKeyboardControl()
    global state
    state = GlobalState(smkc)
    asr = Thread(target=listen_asr, args=tuple())
    asr.daemon = True
    asr.start()

    controller = Thread(target=listen_xbox, args=tuple())
    controller.daemon = True
    controller.start()

    smkc.run()
