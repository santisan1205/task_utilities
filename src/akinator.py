#!/usr/bin/env python3
from transitions import Machine
from task_module import Task_module as tm
import akipy
import rospy
import threading
from robot_toolkit_msgs.msg import speech_recognition_status_msg, animation_msg, motion_tools_msg, leds_parameters_msg, touch_msg
from robot_toolkit_msgs.srv import tablet_service_srv,  set_open_close_hand_srv, set_open_close_hand_srvRequest, motion_tools_srv, battery_service_srv, set_output_volume_srv, tablet_service_srvRequest, set_stiffnesses_srv, set_stiffnesses_srvRequest
import os
import ConsoleFormatter

class Akinator(object):
    def __init__(self) -> None:
        self.consoleFormatter=ConsoleFormatter.ConsoleFormatter()
        self.task_name = "AKINATOR"
        self.hearing = True
        self.new_game = False
        self.answer = ""
        self.question_number = 1

        states =  ["AKINATOR", "INIT", "WAIT4GUEST", "ASK"]
        self.tm = tm(perception = False,speech=True, pytoolkit=True)
        self.tm.initialize_node(self.task_name)
        
        #Definir transiciones
        transitions = [
            {"trigger": "start", "source": "AKINATOR", "dest": "INIT"},
            {"trigger": "begin", "source": "INIT", "dest": "WAIT4GUEST"},
            {"trigger": "new_game_start", "source": "WAIT4GUEST", "dest": "AKINATOR"},
            {"trigger": "finish", "source": "AKINATOR", "dest": "WAIT4GUEST"},
        ]

        self.machine = Machine(model=self, states=states, transitions=transitions, initial='AKINATOR')
        rospy_check = threading.Thread(target=self.check_rospy)
        rospy_check.start()
        

    def on_enter_INIT(self):
        print(self.consoleFormatter.format("Inicializacion del task: "+self.task_name, "HEADER"))
        self.tm.initialize_pepper()
        self.tm.hot_word(["jugar"], thresholds=[0.4],language="Spanish")
        subscriber = rospy.Subscriber("/pytoolkit/ALSpeechRecognition/status",speech_recognition_status_msg,self.callback_hot_word)
        self.tm.motion_tools_service()
        self.tm.enable_breathing_service()
        self.begin()

    def on_enter_WAIT4GUEST(self):
        print(self.consoleFormatter.format("WAIT4GUEST", "HEADER"))
        self.tm.show_web_view(web_url= "http://192.168.0.229:8000/")
        while not self.new_game:
            rospy.sleep(0.1)
        self.tm.play_animation(animation_name="Waiting/Think_3")
        rospy.sleep(1)
        self.tm.talk("Dejame pensar mi primera pregunta", language="Spanish", wait=False)
        print("pre aki")
        self.aki = akipy.Akinator()
        self.aki.start_game("es")
        print("post aki")
        self.tm.talk("Ya estoy lista para jugar!", "Spanish", wait=True)
        #Agregar imprimir en la tablet
        self.new_game_start()

    def on_enter_AKINATOR(self):
        print(self.consoleFormatter.format("AKINATOR", "HEADER"))
        while not self.aki.win:
            question = self.aki.question
            animation = self.convert_akitude_animation(self.aki.akitude)
            print(self.consoleFormatter.format("Pregunta del Akinator: ", "OKGREEN"))
            print(self.aki.question)
            
            self.tm.show_web_view(web_url= f"http://192.168.0.229:8000/static/preguntas.html?pregunta={question}&numero={self.question_number}")
            self.question_number+=1
            self.tm.talk(question, "Spanish",animated=False, wait=True)
            self.answer=""
            self.tm.play_animation(animation_name=animation)
            speech2text_thread = threading.Thread(target=self.speech2text_function)
            speech2text_thread.start()    
            
            while self.answer == "":
                rospy.sleep(0.1)
            self.tm.set_volume(70)
            
            speech2text_thread.join(20)
            
            if "si" in self.answer.lower() or "sí" in self.answer.lower():
                self.aki.answer("yes")
            elif "probablemente no" in self.answer.lower():
                self.aki.answer("probably not")
            elif "no" in self.answer.lower():
                self.aki.answer("no")
            elif "ni idea" in self.answer.lower():
                self.aki.answer("idk")
            elif "probablemente" in self.answer.lower():
                self.aki.answer("probably")
            
            
            if self.aki.win:
                self.tm.talk(f"Tu personaje es:{self.aki.name_proposition}?", "Spanish")
                text = self.tm.speech2text_srv(seconds=0,lang="esp")
                if "si" in text.lower() or "sí" in text.lower():
                    self.tm.show_words_proxy()
                    rospy.sleep(1)
                    self.tm.talk("Gane!",animated=False)
                    self.tm.play_animation(animation_name="Emotions/Positive/Winner_2")
                else:
                    self.tm.talk("Perdi", "Spanish")
                    self.tm.play_animation(animation_name="Emotions/Negative/Sad_1")
                    
                self.new_game = False
                self.finish()
    
    def convert_akitude_animation(self,akitude):
        if not akitude == "None":
            if akitude == "concentration_intense.png":
                return "Gestures/Thinking_3"
            elif akitude == "confiant.png":
                return "Emotions/Positive/Proud_3"
            elif akitude == "deception.png":
                return "Gestures/Thinking_1"
            elif akitude == "defi.png":
                return "Gestures/Thinking_3"
            elif akitude == "etonnement.png":
                return "Gestures/Everything_1"
            elif akitude == "inspiration_forte.png":
                return "Gestures/Hey_10"
            elif akitude == "inspiration_legere.png":
                return "Waiting/Think_4"
            elif akitude == "leger_decouragement.png":
                return "Gestures/IDontKnow_2"
            elif akitude == "mobile.png":
                return "Waiting/LookHand_2"
            elif akitude == "serein.png":
                return "Emotions/Positive/Sure_1"
            elif akitude == "surprise.png":
                return "Emotions/Negative/Disappointed_1"
            elif akitude == "tension.png":
                return "Reactions/TouchHead_4"
            elif akitude == "triomphe.png":
                return "Emotions/Positive/Winner_2"
            elif akitude == "vrai_decouragement.png":
                return "Emotions/Negative/Angry_1"
        return ""
    
    def callback_hot_word(self,data):
        word = data.status
        print(word, " listened")
        if word=="jugar":
            self.new_game = True
        if self.answer=="":
            if word=="si":
                self.answer = "si"
            elif word=="probablemente no":
                self.answer = "probablemente no"
            elif word=="no":
                self.answer = "no"
            elif word=="ni idea":
                self.answer = "ni idea"
            elif word=="probablemente":
                self.answer = "probablemente"
    
    def speech2text_function(self):
        while self.answer=="":
            temporal = self.tm.speech2text_srv(seconds=0,lang="esp")
            if not ("None" in temporal):
                self.answer=temporal
            else:
                self.tm.talk(text="Disculpa, no te entendi. Por favor repite tu respuesta ",language="Spanish", wait=True, animated=False)
                
                

    def check_rospy(self):
        #Termina todos los procesos al cerrar el nodo
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
        print(self.consoleFormatter.format("Shutting down", "FAIL"))
        os._exit(os.EX_OK)

    def run(self):
        while not rospy.is_shutdown():
            self.start()
    
if __name__ == "__main__":
    sm = Akinator()
    sm.run()
    rospy.spin()        
