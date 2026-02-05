#!/usr/bin/env python3
from transitions import Machine
from task_module import Task_module as tm
import ConsoleFormatter
import time
import random
import threading
import sys
import rospy
import math
import os
import numpy as np
import re
import ast
from code_generation import ls_generate as gen
from code_generation import generate_utils 
from code_generation.database.models import Model
from std_msgs.msg import String
from robot_toolkit_msgs.msg import speech_recognition_status_msg

class GPSR(object):
    def __init__(self):
        self.gen = gen.LongStringGenerator()
        # TODO
        """
        - Revisar si se meten los servicios de interface o del pytoolkit directamente (ojala nodo de interface)
        - Falta meter todos los servicios del pytoolkit al modulo para que se puedan llamar facilmente desde la maquina de estados.
        - Falta crear behaviours como el spin_until_object que es usado varias veces en varios tasks.
        - Falta revisar todos los angulos de navegacion al hacer look_4_something y la velocidad de giro 
        - Poner una redundancia para cuando el robot asigna un nuevo a id a una misma persona para que no la presente (lista con los nombres de los que ya presento, incluyendo el actual)
        """

        self.consoleFormatter=ConsoleFormatter.ConsoleFormatter()
        # Definir los estados posibles del semáforo
        self.task_name = "GPSR"
        states = ['INIT', 'WAIT4GUEST', 'GPSR', 'GO2GPSR']
        self.tm = tm(perception = False,speech=False,manipulation=False, navigation=False, pytoolkit=False)
        #self.tm.initialize_node(self.task_name)
        # Definir las transiciones permitidas entre los estados
        transitions = [
            {'trigger': 'start', 'source': 'GPSR', 'dest': 'INIT'},
            {'trigger': 'beggining', 'source': 'INIT', 'dest': 'GO2GPSR'},
            {'trigger': 'go_to_gpsr', 'source': 'GO2GPSR', 'dest': 'WAIT4GUEST'},
            {'trigger': 'person_arrived', 'source': 'WAIT4GUEST', 'dest': 'GPSR'},
            {'trigger': 'GPSR_done', 'source': 'GPSR', 'dest': 'GO2GPSR'}
        ]
        
        # Crear la máquina de estados
        self.machine = Machine(model=self, states=states, transitions=transitions, initial='GPSR')
        rospy_check = threading.Thread(target=self.check_rospy)
        rospy_check.start()
        self.posePublisherSubscriber = rospy.Subscriber(
            "perception_utilities/pose_publisher", String, self.posePublisherCallback
        )
        ############################# GLOBAL VARIABLES #############################
        self.location = "house_door"

    def on_enter_INIT(self):
        print(self.consoleFormatter.format("Inicializacion del task: "+self.task_name, "HEADER"))
        self.tm.initialize_pepper()
        self.beggining()

    def on_enter_GPSR(self):
        print(self.consoleFormatter.format("GPSR", "HEADER"))
        task = input("Inserte comando: ")
        if task!="":
            generate_utils.load_code_gen_config() 
            contador = 0
            while contador<3:
                code = self.gen.generate_code(task,Model.GPT4).replace("`","").replace("python","")
                print(code)
                if not "I am sorry but I cannot complete this task" in code:
                    print("\nIt is possible to execute the request")
                    if self.is_valid_syntax(code):
                        contador = 5
                contador += 1
            if contador==4:
                print("\nIt is NOT possible to execute the request!")
        self.GPSR_done()

    def on_enter_GO2GPSR(self):
        print(self.consoleFormatter.format("GO2GPSR", "HEADER"))
        self.go_to_gpsr()

    def on_enter_WAIT4GUEST(self):
        print(self.consoleFormatter.format("WAIT4GUEST", "HEADER"))
        self.person_arrived()

    def check_rospy(self):
        #Termina todos los procesos al cerrar el nodo
        while not rospy.is_shutdown():
            time.sleep(0.1)
        print(self.consoleFormatter.format("Shutting down", "FAIL"))
        os._exit(os.EX_OK)

    def run(self):
        while not rospy.is_shutdown():
            self.start()
        


    def posePublisherCallback(self, msg):
        if msg.data == "Pointing to the left":
            self.tm.pointing = "left"
        elif msg.data == "Pointing to the right":
            self.tm.pointing = "right"
        elif msg.data=="Right hand up":
            self.tm.pointing = "center"
            self.tm.hand_up = "right"
        elif msg.data=="Left hand up":
            self.tm.pointing = "center"
            self.tm.hand_up = "left"
        else:
            self.tm.pointing = "none"
            self.tm.hand_up = "none"

    def callback_hot_word(self,data):
        word = data.status
        print(word)
        if word == "stop":
            self.tm.follow_you(False)
    
    def is_valid_syntax(self, code):
        try:
            ast.parse(code)
            return True
        except SyntaxError:
            return False
    
# Crear una instancia de la maquina de estados
if __name__ == "__main__":
    sm = GPSR()
    sm.run()
    rospy.spin()
