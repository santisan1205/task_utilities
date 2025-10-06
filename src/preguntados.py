from transitions import Machine
from task_module import task_module as tm
import threading
import time
import rospy
import os
import ConsoleFormatter

class Preguntados(object):
    def __init__(self) -> None:
        self.consoleFormatter=ConsoleFormatter.ConsoleFormatter()
        self.task_name = "PREGUNTADOS"
        self.hearing = True
        self.new_game = False
        self.answer = ""
        self.question_number = 1

        states =  ["PREGUNTADOS", "INIT", "WAIT4GUEST", "ASK", "CHECKANSWER", "SHOWANSWER"]
        self.tm = tm(speech=True, pytoolkit=True)
        self.tm.initialize_node(self.task_name)
        
        # Se definen las transiciones entre estados
        transitions = [
            {"trigger": "start", "source": "PREGUNTADOS", "dest": "INIT"},
            {"trigger": "begin", "source": "INIT", "dest": "WAIT4GUEST"},
            {"trigger": "new_game_start", "source": "WAIT4GUEST", "dest": "PREGUNTADOS"},
            {"trigger": "finish", "source": "PREGUNTADOS", "dest": "WAIT4GUEST"},
            {"trigger": "ask_question", "source": "PREGUNTADOS", "dest": "ASK"},
            {"trigger": "check_answer", "source": "ASK", "dest": "CHECKANSWER"},
            {"trigger": "show_answer", "source": "CHECKANSWER", "dest": "SHOWANSWER"},
        ]
        self.machine = Machine(model=self, states=states, transitions=transitions, initial='PREGUNTADOS')
        rospy_check = threading.Thread(target=self.check_rospy)
        rospy_check.start()
        
    def on_enter_INIT(self):
        print(self.consoleFormatter.format("Inicializacion del task: "+self.task_name, "HEADER"))
        self.tm.initialize_pepper()
        self.tm.talk("Hola, soy Pepper. Vamos a jugar a Preguntados. Te deseo suerte.")
        self.tm.talk("El juego consiste en que yo te hare preguntas de diferentes categorias y tu deberas responderlas.")
        self.begin()
    
    def on_enter_WAIT4GUEST(self):
        print(self.consoleFormatter.format("Esperando a un invitado...", "HEADER"))
        self.tm.talk("Estoy esperando a que un invitado me diga: Empezar juego, para comenzar una nueva partida.")
        while self.hearing:
            text = self.tm.speech2text_srv()
            if text:
                print(self.consoleFormatter.format("He escuchado: "+text, "OKGREEN"))
                if "empezar juego" in text.lower():
                    self.new_game = True
                    self.hearing = False
                    self.tm.say("Perfecto, vamos a empezar una nueva partida.")
                    self.new_game_start()
            