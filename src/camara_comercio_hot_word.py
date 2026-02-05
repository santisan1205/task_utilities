#!/usr/bin/env python3
from transitions import Machine
from task_module import Task_module as tm
from datetime import datetime
import ConsoleFormatter
import threading
import pandas as pd
import rospy
import random
import os
import csv

from std_srvs.srv import SetBool
from robot_toolkit_msgs.msg import speech_recognition_status_msg, animation_msg, motion_tools_msg, leds_parameters_msg, touch_msg
from robot_toolkit_msgs.srv import tablet_service_srv,  set_open_close_hand_srv, set_open_close_hand_srvRequest, motion_tools_srv, battery_service_srv, set_output_volume_srv, tablet_service_srvRequest, set_stiffnesses_srv, set_stiffnesses_srvRequest

class Evento(object):
    def __init__(self):
        
        self.consoleFormatter=ConsoleFormatter.ConsoleFormatter()
        # Definir los estados posibles del semáforo
        self.task_name = "Event"
        self.hearing = False
        self.is_done = False
        self.hey_pepper=False
        
        states = ['INIT', 'WAIT4GUEST', 'TALK']
        self.tm = tm(perception = False,speech=True, pytoolkit=True)
        self.tm.initialize_node(self.task_name)
        # Definir las transiciones permitidas entre los estados
        transitions = [
            {'trigger': 'start', 'source': 'TALK', 'dest': 'INIT'},
            {'trigger': 'beggining', 'source': 'INIT', 'dest': 'WAIT4GUEST'},
            {'trigger': 'person_arrived', 'source': 'WAIT4GUEST', 'dest': 'TALK'},
            {'trigger': 'TALK_done', 'source': 'TALK', 'dest': 'WAIT4GUEST'}
        ]
        
        # Crear la máquina de estados
        self.machine = Machine(model=self, states=states, transitions=transitions, initial='TALK')
        rospy_check = threading.Thread(target=self.check_rospy)
        rospy_check.start()
        ############################# GLOBAL VARIABLES #############################

    def on_enter_INIT(self):
        print(self.consoleFormatter.format("Inicializacion del task: "+self.task_name, "HEADER"))
        self.tm.initialize_pepper()
        subscriber = rospy.Subscriber("/pytoolkit/ALSpeechRecognition/status",speech_recognition_status_msg,self.callback_hot_word)
        self.motion_tools_service()
        self.enable_breathing_service()
        self.enable_hot_word_service()
        self.tm.show_words_proxy()
        self.beggining()

    def on_enter_TALK(self):
        print(self.consoleFormatter.format("TALK", "HEADER"))
        rospy.sleep(0.9)
        self.tm.start_tracker_proxy()
        while not self.is_done:
            if not self.hearing:
                #self.tm.talk("Hola! Bienvenido a ideas que transforman. \\pau=300\\ Sabías que Bogotá está construyendo un Campus de Ciencia, Tecnología e Innovación? Te gustaría saber cómo el Campus de Ciencia, Tecnología e Innovación transformará a Bogotá? \\pau=300\\ Puedes preguntarme qué es el Campus, dónde estará ubicado, cuándo estará en funcionamiento o quiénes están promoviendo este proyecto? \\pau=300\\ Además,  te mostraré dónde estará ubicado y cómo se verá la primera sede! \\pau=300\\ Qué quieres saber?","Spanish",animated=True,wait=True)
                rospy.sleep(20)
            rospy.sleep(0.1)
        self.TALK_done()

    def on_enter_WAIT4GUEST(self):
        self.is_done=False
        print(self.consoleFormatter.format("WAIT4GUEST", "HEADER"))
        self.tm.setRPosture_srv("stand")
        self.tm.setMoveHead_srv.call("up")
        self.tm.stop_tracker_proxy()
        self.person_arrived()

    def check_rospy(self):
        #Termina todos los procesos al cerrar el nodo
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
        print(self.consoleFormatter.format("Shutting down", "FAIL"))
        os._exit(os.EX_OK)

    def run(self):
        while not rospy.is_shutdown():
            self.start()
    
    def enable_hot_word_service(self):
        """
        Enables hotwords detection from the toolkit of the robot.
        """
        print("Waiting for hot word service")
        rospy.wait_for_service('/pytoolkit/ALSpeechRecognition/set_hot_word_language_srv')
        try:
            hot_word_language_srv = rospy.ServiceProxy("/pytoolkit/ALSpeechRecognition/set_hot_word_language_srv", tablet_service_srv)
            hot_word_language_srv("Spanish")
            self.set_hot_words()
            print("Hot word service connected!")
        except rospy.ServiceException as e:
            print("Service call failed")
    
    def enable_breathing_service(self):
        """
        Enables the breathing animations of the robot.
        """
        request = set_open_close_hand_srvRequest()
        request.hand = "All"
        request.state = "True"
        print("Waiting for breathing service")
        rospy.wait_for_service("/pytoolkit/ALMotion/toggle_breathing_srv")
        try:
            toggle_breathing_proxy = rospy.ServiceProxy("/pytoolkit/ALMotion/toggle_breathing_srv", set_open_close_hand_srv)
            toggle_breathing_proxy(request)
            print("Breathing service connected!")
        except rospy.ServiceException as e:
            print("Service call failed")
        request = set_open_close_hand_srvRequest()
        request.hand = "Head"
        request.state = "False"
        print("Waiting for breathing service")
        rospy.wait_for_service("/pytoolkit/ALMotion/toggle_breathing_srv")
        try:
            toggle_breathing_proxy = rospy.ServiceProxy("/pytoolkit/ALMotion/toggle_breathing_srv", set_open_close_hand_srv)
            toggle_breathing_proxy(request)
            print("Breathing service connected!")
        except rospy.ServiceException as e:
            print("Service call failed")
    
    def motion_tools_service(self):
        """
        Enables the motion Tools service from the toolkit of the robot.
        """
        request = motion_tools_msg()
        request.command = "enable_all"
        print("Waiting for motion tools service")
        rospy.wait_for_service('/robot_toolkit/motion_tools_srv')
        try:
            motion = rospy.ServiceProxy('/robot_toolkit/motion_tools_srv', motion_tools_srv)
            motion(request)
            print("Motion tools service connected!")
        except rospy.ServiceException as e:
            print("Service call failed")
            

    def set_hot_words(self):
        self.tm.hot_word(["que","dónde","cuando","quienes"],thresholds=[0.4, 0.42,0.35,0.35])
    
    
    def setLedsColor(self, r,g,b):
        """
        Function for setting the colors of the eyes of the robot.
        Args:
        r,g,b numbers
            r for red
            g for green
            b for blue
        """
        ledsPublisher = rospy.Publisher('/leds', leds_parameters_msg, queue_size=10)
        ledsMessage = leds_parameters_msg()
        ledsMessage.name = "FaceLeds"
        ledsMessage.red = r
        ledsMessage.green = g
        ledsMessage.blue = b
        ledsMessage.time = 0
        ledsPublisher.publish(ledsMessage)  #Inicio(aguamarina), Pepper esta ALSpeechRecognitionStatusPublisherlista para escuchar
    
    def callback_hot_word(self,data):
        word = data.status
        print(word, "listened")
            
        if not self.hearing:
            self.hearing = True
            answer = ""
            if word == "que":
                answer = "Me gusta tu pregunta! \\pau=300\\ El Campus será un generador de conocimiento y lugar de encuentro de sociedad, academia, sector público y privado para crear soluciones innovadoras que realmente ayuden a resolver problemas de ciudad y de las empresas. \\pau=300\\ Te interesa ver dónde estará ubicado y cómo ver cómo será la primera sede? ¡Vamos!"
            
            elif word == "dónde":
                answer = "Que bueno que preguntas! \\pau=300\\  Imagina un espacio de 247 hectáreas  y un edificio de 44mil metros cuadrados y 23 pisos que serán el epicentro de la innovación en Bogotá. \\pau=300\\  Esa es la futura casa del Campus y es muy cerca de aquí. \\pau=300\\  Un lugar pensado para intercambiar ideas y transformar a Bogotá. \\pau=300\\ Quieres ver exactamente dónde estará y cómo será la primera sede? \\pau=300\\ Vamos!"
            
            elif word == "cuando":
                answer = "Gran pregunta!  \\pau=300\\ La construcción de la primera sede, un edificio de 44mil metros cuadrados empieza el próximo año, y abrirá sus puertas en dos años y medio.  \\pau=300\\ No falta mucho para ver este sueño de ciudad convertido en una realidad. \\pau=300\\ Quieres ver esa primera sede? \\pau=300\\ Vamos!"
            
            elif word == "quienes":
                answer = "Gracias por preguntar!  \\pau=300\\ Esta es una iniciativa liderada por la Cámara de Comercio de Bogotá, en asocio con la Alcaldía Mayor a través de la secretaría de desarrollo económico y Atenea, con Corferias, y  con las tres principales cajas de compensación: Cafam, Compensar y Colsubsidio, además del SENA.  \\pau=300\\ Te gustaría ver la ubicación y el diseño de la sede inicial?  \\pau=300\\ Acércate y te lo enseño!"
            self.tm.talk(answer,"Spanish",animated=True, wait=True)
            rospy.sleep(1)
            self.hearing = False
    
# Crear una instancia de la maquina de estados
if __name__ == "__main__":
    sm = Evento()
    sm.run()
    rospy.spin()
