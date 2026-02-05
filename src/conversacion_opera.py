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
        self.task_name = "Event"
        self.hearing = True
        self.is_done = False
        self.hey_pepper=False
        self.haciendo_animacion = False
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
        print(self.consoleFormatter.format("Waiting for pytoolkit/ALTabletService/show_picture_srv...", "WARNING"))
        rospy.wait_for_service('pytoolkit/ALTabletService/show_picture_srv')
        print(self.consoleFormatter.format("Waiting for /pytoolkit/ALTabletService/show_image_srv...", "WARNING"))
        rospy.wait_for_service("/pytoolkit/ALTabletService/show_image_srv")
        self.show_image_srv = rospy.ServiceProxy("/pytoolkit/ALTabletService/show_image_srv",tablet_service_srv)
        self.show_picture_proxy = rospy.ServiceProxy('pytoolkit/ALTabletService/show_picture_srv', battery_service_srv)
        self.play_dance_srv = rospy.ServiceProxy("/pytoolkit/ALMotion/play_dance_srv", set_output_volume_srv)
        self.animationPublisher = rospy.Publisher('/animations', animation_msg, queue_size=10)
        self.motion_set_stiffnesses_client = rospy.ServiceProxy("pytoolkit/ALMotion/set_stiffnesses_srv", set_stiffnesses_srv)
        req_stiffnesses = set_stiffnesses_srvRequest()
        req_stiffnesses.names = "Body"
        req_stiffnesses.stiffnesses = 1
        self.motion_set_stiffnesses_client(req_stiffnesses)
        self.set_arms_security = rospy.ServiceProxy("pytoolkit/ALMotion/set_arms_security_srv", SetBool)
        self.set_arms_security(True)
        self.armsSensorSubscriber = rospy.Subscriber("/touch", touch_msg, self.callback_arms_sensor_subscriber)
        rospy.wait_for_service("/pytoolkit/ALTabletService/show_image_srv")
        self.beggining()

    def on_enter_TALK(self):
        print(self.consoleFormatter.format("TALK", "HEADER"))
        rospy.sleep(0.9)
        self.hearing = True
        while not self.is_done:
            if self.hey_pepper:
                self.tm.show_words_proxy()
                self.hey_pepper_function()
                self.hey_pepper=False
            rospy.sleep(0.1)
        self.tm.talk("Adios","Spanish",animated=True)
        self.TALK_done()

    def on_enter_WAIT4GUEST(self):
        self.tm.show_words_proxy()
        self.is_done=False
        print(self.consoleFormatter.format("WAIT4GUEST", "HEADER"))
        self.tm.setRPosture_srv("stand")
        self.tm.set_angles_srv(["HeadYaw","HeadPitch"],[0,0.6],0.05)
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
        self.tm.hot_word([])
        self.tm.talk("Escuchando","Spanish",animated=True,wait=False)
        rospy.sleep(1)
        text = self.tm.speech2text_srv(seconds=0,lang="esp")
        anim_msg = self.gen_anim_msg("Waiting/Think_3")
        self.animationPublisher.publish(anim_msg)
        if not ("None" in text):
            request = f"""Nova dijo: {text}. Justo ahora estas hablando con el robot Pepper Nova de la Universidad de los Andes. Responde brevemente. Si hay palabras en otro idioma en tu respuesta escribelas como se pronunicarian en español porque en este momento solo puedes hablar español y ningun otro idioma, por ejemplo si en tu respuesta esta Python, responde Paiton. No añadas contenido complejo a tu respuesta como codigo, solo explica lo que sea necesario."""
            answer=self.tm.answer_question(request, save_conversation=True) 
            self.tm.talk("Hey Nova \\pau=4000\\"+answer,"Spanish",animated=True, wait=True, speed="90")
        else:
            self.tm.talk("Hey Nova \\pau=4000\\"+"Disculpa, no te entendi, puedes hablar cuando mis ojos esten azules. Por favor habla mas lento","Spanish",animated=True, speed="90")
        self.set_hot_words()

    def set_hot_words(self):
        if self.hearing:
            self.tm.hot_word(["hey opera"],thresholds=[0.25])
            

    def callback_arms_sensor_subscriber(self, msg: touch_msg):
        req_stiffnesses = set_stiffnesses_srvRequest()
        if "hand_left_back" in msg.name:
            req_stiffnesses.names = "LArm"
        elif "hand_right_back" in msg.name:  
            req_stiffnesses.names = "RArm"
        req_stiffnesses.stiffnesses = 0 if msg.state else 1
        self.motion_set_stiffnesses_client(req_stiffnesses)
        if msg.state and not "head" in msg.name:
            self.tm.hot_word([])
            self.tm.talk("Sosten firmemente ahi para mover mi brazo", language="Spanish", wait=True)
            self.set_hot_words()
        elif not "head" in msg.name:
            self.tm.hot_word([])
            self.tm.talk("Soltaste mi mano", language="Spanish", wait=True)
            self.set_hot_words()

    def gen_anim_msg(self, animation):
        anim_msg = animation_msg()
        anim_msg.family = "animations"
        anim_msg.animation_name = animation
        return anim_msg
    
    def dance_threadf(self, number):
        #Se ponen los bailes en un thread porque estancan la ejecucion y no permiten checkear otras hotwords, como detente
        self.haciendo_animacion = True
        self.play_dance_srv(number)
        self.haciendo_animacion = False
    
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
        if word=="detente":
            self.tm.setRPosture_srv("stand")
            print("se detuvo al robot")
            rospy.sleep(2)
            self.haciendo_animacion = False
            
        if not self.haciendo_animacion:
            self.haciendo_animacion = True
            if word == "chao":
                self.is_done = True
                self.tm.answer_question("", save_conversation=False) 
            elif word == "hey opera":
                self.hey_pepper = True
            elif word == "guitarra":
                anim_msg = self.gen_anim_msg("Waiting/AirGuitar_1")
                self.animationPublisher.publish(anim_msg)
            elif word == "besos":
                anim_msg = self.gen_anim_msg("Gestures/Kisses_1")
                self.animationPublisher.publish(anim_msg)
                self.tm.talk("Muah!","Spanish")
            elif word == "baile":
                dance_thread = threading.Thread(target=self.dance_threadf,args=[1])
                dance_thread.start()
            elif word == "asereje":
                dance_thread = threading.Thread(target=self.dance_threadf,args=[3])
                dance_thread.start()
            elif word == "pose":
                anim_msg = self.gen_anim_msg("Gestures/ShowSky_8")
                self.animationPublisher.publish(anim_msg)
            elif word == "foto":
                anim_msg = self.gen_anim_msg("Waiting/TakePicture_1")
                self.animationPublisher.publish(anim_msg)
                rospy.sleep(2)
                self.show_picture_proxy()
            elif word == "cumpleaños":
                anim_msg = self.gen_anim_msg("Waiting/HappyBirthday_1")
                self.animationPublisher.publish(anim_msg)
            elif word == "corazon":
                anim_msg = self.gen_anim_msg("Waiting/LoveYou_1")
                self.animationPublisher.publish(anim_msg)
            elif word == "llama":
                anim_msg = self.gen_anim_msg("Waiting/CallSomeone_1")
                self.animationPublisher.publish(anim_msg)
            elif word == "helicoptero":
                anim_msg = self.gen_anim_msg("Waiting/Helicopter_1")
                self.animationPublisher.publish(anim_msg)
            elif word == "zombi":
                anim_msg = self.gen_anim_msg("Waiting/Zombie_1")
                self.animationPublisher.publish(anim_msg)
            elif word == "carro":
                anim_msg = self.gen_anim_msg("Waiting/DriveCar_1")
                self.animationPublisher.publish(anim_msg)
            elif word == "musculos":
                anim_msg = self.gen_anim_msg("Waiting/ShowMuscles_3")
                self.animationPublisher.publish(anim_msg)
            elif word == "gracias":
                anim_msg = self.gen_anim_msg("Gestures/BowShort_3")
                self.animationPublisher.publish(anim_msg)
            if word!="baile" and word!="asereje":
                self.haciendo_animacion = False
    
# Crear una instancia de la maquina de estados
if __name__ == "__main__":
    sm = Evento()
    sm.run()
    rospy.spin()
