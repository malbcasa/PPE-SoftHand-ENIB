#!/usr/bin/env python3
# license removed for brevity

#defining tags for the motor state
ACTIVATED = 1
DEACTIVATED = 0

import rospy
import subprocess
import os, signal, sys
from cmd import Cmd
from handController import HandController
import time
import psutil

class Qbhand_cmd(Cmd):


    def __init__(self):
        super().__init__()
        self.node = HandController()
        self.motorState = DEACTIVATED
        os.system('clear')
        self.intro = "Qb Robotics Soft Hand Research cmd réalisé par les étudiants de l'ENIB\n"
        self.intro += "Type help or ? to list commands.\n"
        self.prompt = ">> "
        self.default_velocity = 1.0
     

    def do_activate_motors(self,args):
        '''
        In order to be able to command the hand, the motor must be
        activated first. Therefore, this is the first command to
        type after a succesful connection with a QB Robotics Soft
        Hand Device. It takes no arguments.
        '''
        self.node.activate_motors()
        self.motorState = ACTIVATED

    def do_deactivate_motors(self,args=""):
        '''
        Disables the ability to give position commands to the hand.
        After this, only no position related commands should be
        called. It takes no arguments.
        '''
        self.node.deactivate_motors()
        self.motorState = DEACTIVATED

    def do_get_measurements(self,args):
        """
        It takes no arguments and shows the current and position
        measured by the hand.
        """
        self.node.get_measurements()
    
    def do_get_closure_rate(self,args):
        """
        It takes no arguments and shows the current closure rate
        of the hand.
        """
        print(self.node.get_closure_rate())
    
    def do_set_default_velocity(self, arg):
        """
        Sets the default velocity for the position commands.

        Parameters:
            arg (float): Desired default velocity.
        """
        self.default_velocity = float(arg)

    def do_set_closure(self, args):
        """
        Send a position command to the hand.

        Parameters:
            position (float): Value between 0.0(fully open) to 1.0 (fully closed)
            time (float): Desired amount of time for the hand to go from actual
            to desired position.
        """
        if self.motorState == DEACTIVATED:
            print("ERROR: The motors are deactivated. Please activate them before sending a position command.")
            return
        args_list = args.split()
        print(args_list)
        if len(args_list) < 2:
            args_list.append(self.default_velocity)
        self.node.set_closure(float(args_list[0]),float(args_list[1]))

    def do_manual_mode(self, args):
        """
         Manual mode to control the hand from the terminal, with the commands
         'z' and 's' to open and close the hand, and 'q' to quit the program
         """
        if self.motorState == DEACTIVATED:
            print("ERROR: The motors are deactivated. Please activate them before using the manual mode.")
            return
        self.node.manual_mode()

    def do_fully_open(self, arg=None):
        """
        Completely opens the hand.

        Parameters:
            arg (float, optional): Desired amount of time for the hand to go from actual
            to fully open. If not provided, defaults to 1.0.
        """
        if arg is None  or arg == "":
            arg = self.default_velocity

        if self.motorState == DEACTIVATED:
            print("ERROR: The motors are deactivated. Please activate them before sending a position command.")
            return
        else:
            self.node.set_closure(0.0, float(arg))


    def do_fully_close(self, arg=None):
        """
        Completely closes the hand.

        Parameters:
            arg (float, optional): Desired amount of time for the hand to go from actual
            to fully closed. If not provided, defaults to 1.0.
        """
        if arg is None  or arg == "":
            arg = self.default_velocity

        if self.motorState == DEACTIVATED:
            print("ERROR: The motors are deactivated. Please activate them before sending a position command.")
            return

        self.node.set_closure(1.0, float(arg))


    def postcmd(self, stop, line):
        print("\n")
        return stop

    def do_exit(self,line="\n"):
        """
        Deactivates the motor, then kills the hand ROS node
        and finish the program
        """
        self.do_deactivate_motors()
        self.node.kill_node()
        try:
            pid = None
            for proc in psutil.process_iter(['pid', 'name']):
                    if proc.info['name'] == "gnome-terminal-server":
                        pid = proc.info['pid']
                       
            subprocess.run(["kill", "-9", str(pid)])
        except:
            pass
        sys.exit("Cmd program finished.")

if __name__ == '__main__':
    try:
        launch = subprocess.Popen(['gnome-terminal', '--', 'bash', '-c',"roslaunch qb_hand_control control_qbhand.launch standalone:=true activate_on_initialization:=false device_id:=1 use_rviz:=true"])
        Qbhand_cmd().cmdloop()
    except rospy.ROSInterruptException:
        pass