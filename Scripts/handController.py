#!/usr/bin/env python3
from pynput import keyboard
import tty
import termios
import sys
import time
import subprocess
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
from qb_device_srvs.srv import GetMeasurements

#Create a class to control the hand
class HandController:
    def __init__(self):
        self.pub_topic = '/qbhand1/control/qbhand1_synergy_trajectory_controller/command'
        rospy.init_node('hand_controller_node')
        self.rate = rospy.Rate(10) 
        self.flag = True
        self.max_closure = 1.0
        self.min_closure = 0.0
        self.get_measurements(True)
       
    def activate_motors(self):
        try:
            subprocess.run(["rosservice", "call", "/communication_handler/activate_motors", "1", "3"])
            self.pub = rospy.Publisher(self.pub_topic, JointTrajectory, queue_size=10)
            self.get_measurements(True)
        except Exception as e:
            print("Error: ", e)
            return
        return
   
    def deactivate_motors(self):
        try:
            subprocess.run(["rosservice", "call", "/communication_handler/deactivate_motors", "1", "3"])
            self.pub.unregister()
        except Exception as e:
            print("Error: ", e)
            return
        return
    
    def get_closure_rate(self):
        return self.closure_rate

    def get_measurements(self,flag=False):
        """try:
            subprocess.run(["rosservice", "call", "/communication_handler/get_measurements", "1", "3", "true", "true","true","true"])
        except Exception as e:
            print("Error: ", e)
            return
        return"""
        rospy.wait_for_service('/communication_handler/get_measurements')
        try:
            get_measurements = rospy.ServiceProxy('/communication_handler/get_measurements', GetMeasurements)
            response = get_measurements(1, 3, True, True, True, True)  # send the request with the parameters

            if response.success:
                if flag:
                    #print("Current Position: ", response.positions[0])
                    self.closure_rate = response.positions[0]/18600
                else:
                    print("Measurements:")
                    print("Positions:", response.positions)
                    print("Currents:", response.currents)
                    print("Commands:", response.commands)
                    print("Timestamp:", response.stamp)
            else:
                print("Error:", response.failures, "failures occurred")
        except rospy.ServiceException as e:
            print("Error:", e)

    def set_closure(self, position, time):
        # Create the message JointTrajectory
        trajectory_msg = JointTrajectory()
       
        # Configure the header of the message
        trajectory_msg.header = Header()
        trajectory_msg.header.seq = 0
        trajectory_msg.header.stamp = rospy.Time.now()
        trajectory_msg.header.frame_id = ''
       
        # Define the names of the joints
        trajectory_msg.joint_names = ['qbhand1_synergy_joint']
       
        # Create a JointTrajectoryPoint
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.velocities = [0.0]
        point.accelerations = [0.0]
        point.effort = [0.0]
        point.time_from_start = rospy.Duration(time)

        # Add the point to the JointTrajectory message
        trajectory_msg.points.append(point)

        for _ in range(3):
           
            # Publish the message
            self.pub.publish(trajectory_msg)
           
            # Debug information
            #rospy.loginfo("Published trajectory message: %s", trajectory_msg)
       
            # Wait until the next iteration
            self.rate.sleep()
        self.closure_rate = position

    def manual_mode(self):
        print("Press 'up' to open, 'down' to close the hand, and 'q' to quit.")

        # Variables para manejar el estado de las teclas
        up_pressed = False
        down_pressed = False
        quit_pressed = False

        # Configuración del terminal para modo no canónico (modo crudo)
        orig_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        # Función para manejar cuando una tecla se presiona
        def on_press(key):
            nonlocal up_pressed, down_pressed, quit_pressed
            try:
                if key.char == 'z':
                    up_pressed = True
                elif key.char == 's':
                    down_pressed = True
                elif key.char == 'q':
                    quit_pressed = True
            except AttributeError:
                pass

        # Función para manejar cuando una tecla se libera
        def on_release(key):
            nonlocal up_pressed, down_pressed, quit_pressed
            try:
                if key.char == 'z':
                    up_pressed = False
                elif key.char == 's':
                    down_pressed = False
                elif key.char == 'q':
                    quit_pressed = False
            except AttributeError:
                pass

        # Crear y empezar el listener
        listener = keyboard.Listener(on_press=on_press, on_release=on_release, suppress=True)
        listener.start()

        try:
            while not rospy.is_shutdown() and not quit_pressed:
                if up_pressed:
                    if self.closure_rate >= self.max_closure:
                        print("Maximum closure reached.")
                    else:
                        self.closure_rate = min(self.closure_rate + 0.1, self.max_closure)
                        self.set_closure(self.closure_rate, 0.1)
                if down_pressed:
                    if self.closure_rate <= self.min_closure:
                        print("Minimum closure reached.")
                    else:
                        self.closure_rate = max(self.closure_rate - 0.1, self.min_closure)
                        self.set_closure(self.closure_rate, 0.1)
                time.sleep(0.1)  # Modera la velocidad de respuesta
        finally:
            # Detener el listener y restaurar la configuración original del terminal
            listener.stop()
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, orig_settings)
            print("\nExiting manual mode.")

    def fully_open(self, time=1.0):
        self.set_closure(0.0, time)

    def fully_close(self, time=1.0):
        self.set_closure(1.0, time)

    def kill_node(self):
        rospy.signal_shutdown("Node killed")



