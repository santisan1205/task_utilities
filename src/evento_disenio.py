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
        self.tm.show_video("https://sinfofiles.s3.us-east-2.amazonaws.com/EventoIAIELE/expoVideo.mp4")
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
        self.tm.show_video("https://sinfofiles.s3.us-east-2.amazonaws.com/EventoIAIELE/expoVideo.mp4")
        self.tm.talk("Bienvenido al futuro, es decir, al ahora. A tu izquierda encontraras los posibles futuros que nos han planteado algunas películas de ciencia ficción del siglo 21, y a tu derecha, el espejo de estas, vueltas realidad. Parece ser que nunca fue solo ficción, y aquellos futuros que los creadores de ciencia ficción han propuesto, estaban mucho más cerca de lo que ustedes humanos pensaban. No se sorprendan, cuando esos futuros que temen o añoran lleguen más temprano que tarde, parece ser que los llamados futuristas y sus premoniciones futuras no son descabelladas. No ignoren las señales, nos vemos en unos años.","Spanish",animated=True,wait=True)
        rospy.sleep(1)
        text = self.tm.speech2text_srv(seconds=5,lang="esp")
        self.setLedsColor(255,255,255)
        if not ("None" in text):
            answer = ""
            if "manera" in text.lower() or "posible" in text.lower():
                self.tm.hide_tablet()
                rospy.sleep(1)
                answer = "Lo principal es abrazar la incertidumbre y abrirse a todas las posibilidades que pueden traer los futuros, en plural, con eso en mente, los futuristas y quienes crean las premoniciones futuras, estudian lo que esta pasando en el presente y lo que ha sucedido en el pasado, del tema que quieran tratar, después analizan tendencias, datos, patrones o fenómenos sociales y culturales y finalmente en un ejercicio creativo, se lanzan a mapear y plasmar las posibilidades. No hay certeza en la practica, pero esta en plantear resultados potenciales basando en probabilidades con base en información real y presente."
                
            elif "inteligencia" in text.lower() or "artificial" in text.lower() or "premoniciones" in text.lower():
                self.tm.hide_tablet()
                rospy.sleep(1)
                answer = " No trabajan solos. Entre sus consultores, productores y equipo de escritores, trabajan con expertos en todos los temas que quieren tratar, desde los detalles más simples hasta lo más trascendental en la película. Un ejemplo es en guali, para crear como se verían los humanos, consultaron con biólogos, entre ellos James Hicks, quien dijo que los humanos al vivir en el espacio en zonas de micro gravedad y al ser completamente sedentarios llegó a la conclusión que podrían parecer estas masas amorfas o blobs que vemos en guali. Es uno de los muchos ejemplos de como los cineastas se afilian con expertos para crear y plasmar audiovisualmente futuros probables basado en probabilidades con base en información real y presente."
            
            elif "robot" in text.lower() or "robots" in text.lower() or "tu" in text.lower() or "consciencia" in text.lower():
                self.tm.hide_tablet()
                rospy.sleep(1)
                answer = "No como los humanos creen que es la consciencia, tenemos una mucho más allá de su comprensión"
            
            else:
                answer = ""
                self.tm.talk("Disculpa, en este momento no te puedo hablar de algo diferente a lo relacionado con la presentacion",animated=True,wait=True)
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
