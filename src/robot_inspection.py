#!/usr/bin/env python3
from transitions import Machine
from task_module import Task_module as tm
import ConsoleFormatter
import time
from threading import Thread
import rospy
import os
import numpy as np

from navigation_msgs.srv import constant_spin_srv
from navigation_msgs.msg import simple_feedback_msg
from robot_toolkit_msgs.srv import set_security_distance_srv, tablet_service_srv
from robot_toolkit_msgs.msg import animation_msg, touch_msg
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion

class ROBOT_INSPECTION(object):
    def __init__(self):
        
        # TODO
        """
        - Revisar si se meten los servicios de interface o del pytoolkit directamente (ojala nodo de interface)
        - Falta meter todos los servicios del pytoolkit al modulo para que se puedan llamar facilmente desde la maquina de estados.
        - Falta crear behaviours como el spin_until_object que es usado varias veces en varios tasks.
        - Falta revisar todos los angulos de navegacion al hacer look_4_something y la velocidad de giro 
        - Poner una redundancia para cuando el robot asigna un nuevo a id a una misma persona para que no la presente (lista con los nombres de los que ya presento, incluyendo el actual)
        """

        self.consoleFormatter=ConsoleFormatter.ConsoleFormatter()
        self.initial_place = "init_ri"
        self.inspector_place = "kitchen"
        self.exit_place = "bedroom"
        self.head_touched = True
        
        self.task_name = "robot inspection"
        states = ['INIT', 'GO2INSPECTION', 'WAIT4TOUCH', 'GO2EXIT']
        self.tm = tm(perception=False, speech=True, navigation=True, pytoolkit=True)
        self.tm.initialize_node("robot_inspection")
        # Definir las transiciones permitidas entre los estados
        transitions = [
            {'trigger': 'start', 'source': 'ROBOT_INSPECTION', 'dest': 'INIT'},
            {'trigger': 'beggining', 'source': 'INIT', 'dest': 'GO2INSPECTION'},
            {'trigger': 'waiting', 'source': 'GO2INSPECTION', 'dest': 'WAIT4TOUCH'},
            {'trigger': 'touched', 'source': 'WAIT4TOUCH', 'dest': 'GO2EXIT'}
        ]
        
        # Crear la máquina de estados
        self.machine = Machine(model=self, states=states, transitions=transitions, initial='ROBOT_INSPECTION')

        rospy_check = Thread(target=self.check_rospy)
        rospy_check.start()

        # ROS Callbacks
        print(self.consoleFormatter.format("Waiting for /touch", "WARNING"))
        subscriber_touch = rospy.Subscriber("/touch", touch_msg, self.callback_head_sensor_subscriber)

        print(self.consoleFormatter.format("Waiting for pytoolkit/ALMotion/set_orthogonal_security_distance_srv...", "WARNING"))
        rospy.wait_for_service("/pytoolkit/ALMotion/set_orthogonal_security_distance_srv")
        self.set_orthogonal_security_srv = rospy.ServiceProxy("/pytoolkit/ALMotion/set_orthogonal_security_distance_srv",set_security_distance_srv)

        print(self.consoleFormatter.format("Waiting for pytoolkit/ALMotion/set_tangential_security_distance_srv...", "WARNING"))
        rospy.wait_for_service("/pytoolkit/ALMotion/set_tangential_security_distance_srv")
        self.set_tangential_security_srv = rospy.ServiceProxy("/pytoolkit/ALMotion/set_tangential_security_distance_srv",set_security_distance_srv)

        ##################### ROS CALLBACK VARIABLES #####################
        self.head_touched = False
    
    def callback_head_sensor_subscriber(self, msg: touch_msg):
        if "head" in msg.name:
            self.head_touched = msg.state
                
    def on_enter_INIT(self):
        self.tm.initialize_pepper() 
        self.tm.talk("I am going to perform the "+ self.task_name,"English")
        print(self.consoleFormatter.format("Inicializacion del task: "+self.task_name, "HEADER"))
        self.tm.set_current_place(self.initial_place)
        self.beggining()
                
    def on_enter_GO2INSPECTION(self):
        print(self.consoleFormatter.format("GO2INSPECTION", "HEADER"))
        self.tm.talk("I am going to the inspection position","English",wait=False)
        self.tm.go_to_place(self.inspector_place)
        self.tm.talk("I'm in the inspection position","English",wait=False)
        self.waiting()
    
    def on_enter_WAIT4TOUCH(self):
        self.tm.talk("Please touch my head","English",wait=False)
        while not self.head_touched:
            rospy.sleep(0.1)
        self.tm.talk("My head has been touched","English",wait=False)
        self.touched()

    def on_enter_GO2EXIT(self):
        print(self.consoleFormatter.format("GO2EXIT", "HEADER"))
        self.tm.talk("I am going to the exit position","English",wait=False)
        # ask_for_robot_path_clearance_thread = Thread(target=self.ask_for_robot_path_clearance)
        # ask_for_robot_path_clearance_thread.start()
        self.tm.go_to_place(self.exit_place)
        self.tm.talk("Robot inspection completed","English", wait=False)
        os._exit(os.EX_OK)
        
    def check_rospy(self):
        #Termina todos los procesos al cerrar el nodo
        while not rospy.is_shutdown():
            time.sleep(0.1)
        print(self.consoleFormatter.format("Shutting down", "FAIL"))
        os._exit(os.EX_OK)

    def run(self):
        while not rospy.is_shutdown():
            self.start()
        
    
# Crear una instancia de la maquina de estados
if __name__ == "__main__":
    sm = ROBOT_INSPECTION()
    sm.run()
    rospy.spin()
