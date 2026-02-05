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
import time

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
        self.last_message_time = time.time()
        
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
        self.motion_tools_service()
        self.enable_breathing_service()
        self.enable_hot_word_service()
        self.beggining()

    def on_enter_TALK(self):
        print(self.consoleFormatter.format("TALK", "HEADER"))
        rospy.sleep(0.9)
        self.tm.show_image("https://sinfofiles.s3.us-east-2.amazonaws.com/imagen_camara.png")
        while not self.is_done:

            self.tm.wait_for_head_touch(timeout=10000000,message="",message_interval=35)
            self.hey_pepper_function()
            self.hey_pepper=False
            rospy.sleep(0.1)
            
        self.tm.talk("Adios","Spanish",animated=True)
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
            
    
    def hey_pepper_function(self):
        self.hearing = True
        self.tm.hide_tablet()
        rospy.sleep(1)
        self.tm.show_video("https://sinfofiles.s3.us-east-2.amazonaws.com/Videos/principal.mp4")
        self.tm.talk("Hola! Bienvenido a ideas que transforman. \\pau=300\\ Sabías que Bogota está construyendo un Campus de Ciencia, Tecnología e Innovación? Te gustaría saber cómo el Campus de Ciencia, Tecnología e Innovación transformará a Bogota? \\pau=300\\ Puedes preguntarme qué es el Campus, dónde estará ubicado, cuándo estará en funcionamiento o quiénes están promoviendo este proyecto? \\pau=300\\ Además,  te mostraré dónde estará ubicado y cómo se verá la primera sede! \\pau=300\\ Qué quieres saber?","Spanish",animated=True,wait=True)
        rospy.sleep(1)
        text = self.tm.speech2text_srv(seconds=7,lang="esp")
        self.setLedsColor(255,255,255)
        if not ("None" in text):
            answer = ""
            if "que" in text.lower() or "qué" in text.lower():
                self.tm.hide_tablet()
                rospy.sleep(1)
                self.tm.show_video("https://sinfofiles.s3.us-east-2.amazonaws.com/Videos/part1.mp4")
                answer = "Me gusta tu pregunta! \\pau=300\\ El Campus será un generador de conocimiento y lugar de encuentro de sociedad, academia, sector público y privado para crear soluciones innovadoras que realmente ayuden a resolver problemas de ciudad y de las empresas."
                
            elif "donde" in text.lower() or "dónde" in text.lower():
                self.tm.hide_tablet()
                rospy.sleep(1)
                self.tm.show_video("https://sinfofiles.s3.us-east-2.amazonaws.com/Videos/parte2.mp4")
                answer = "Que bueno que preguntas! \\pau=300\\  Imagina un espacio de 247 hectáreas  y un edificio de 44mil metros cuadrados y 23 pisos que serán el epicentro de la innovación en Bogotá. \\pau=300\\  Esa es la futura casa del Campus y es muy cerca de aquí. \\pau=300\\  Un lugar pensado para intercambiar ideas y transformar a Bogotá."
            
            elif "cuando" in text.lower() or "cuándo" in text.lower() or "cuanto" in text.lower() or "cuánto" in text.lower():
                self.tm.hide_tablet()
                rospy.sleep(1)
                self.tm.show_video("https://sinfofiles.s3.us-east-2.amazonaws.com/Videos/part3.mp4")
                answer = "Gran pregunta!  \\pau=300\\ La construcción de la primera sede, un edificio de 44mil metros cuadrados empieza el próximo año, y abrirá sus puertas en dos años y medio.  \\pau=300\\ No falta mucho para ver este sueño de ciudad convertido en una realidad. \\pau=300\\   \\pau=300\\"
            
            elif "quienes" in text.lower() or "quiénes" in text.lower():
                self.tm.hide_tablet()
                rospy.sleep(1)
                self.tm.show_video("https://sinfofiles.s3.us-east-2.amazonaws.com/Videos/parte4.mp4")
                answer = "Gracias por preguntar!  \\pau=300\\ Esta es una iniciativa liderada por la Cámara de Comercio de Bogotá, en asocio con la Alcaldía Mayor a través de la secretaría de desarrollo económico y Atenea, con Corferias, y  con las tres principales cajas de compensación: Cafam, Compensar y Colsubsidio, además del SENA.  \\pau=300\\  \\pau=300\\"
            else:
                answer = ""
                self.tm.talk("Disculpa, en este momento solo puedo hablarte de cómo el Campus de Ciencia, Tecnología e Innovación transformará a Bogota. \\pau=300\\ Puedes preguntarme qué es el Campus, dónde estará ubicado, cuándo estará en funcionamiento o quiénes están promoviendo este proyecto? \\pau=300\\ Además,  te mostraré dónde estará ubicado y cómo se verá la primera sede!", "Spanish",animated=True,wait=True)
            self.tm.talk(answer,"Spanish",animated=True, wait=True)
        else:
            self.tm.talk("Disculpa, no te entendi, puedes hablar cuando mis ojos esten azules. Por favor habla mas lento","Spanish",animated=True)
            text = self.tm.speech2text_srv(seconds=7,lang="esp")
            self.setLedsColor(255,255,255)
            
        self.hearing = False

    def set_hot_words(self):
        #self.tm.hot_word(["hey nova"],thresholds=[0.32])
        pass
    
    
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
    
# Crear una instancia de la maquina de estados
if __name__ == "__main__":
    sm = Evento()
    sm.run()
    rospy.spin()
