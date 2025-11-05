#!/usr/bin/env python3
from transitions import Machine
from task_module import Task_module as tm
import threading
import rospy
import time
import os
import ConsoleFormatter
import random
import json
from robot_toolkit_msgs.msg import speech_recognition_status_msg
class Preguntados(object):
    
    def __init__(self) -> None:
        """Clase que representa el juego de Preguntados implementado como una máquina de estados.
        Atributos:
            consoleFormatter (ConsoleFormatter): Formateador de texto para consola.
            task_name (str): Nombre del task.
            tm (task_module): Módulo de tareas para interacción con el robot.
            qas (dict): Diccionario con preguntas y respuestas.
        """
        self.consoleFormatter=ConsoleFormatter.ConsoleFormatter()
        self.task_name = "PREGUNTADOS"
        self.hearing = True
        states =  ["PREGUNTADOS", "INIT", "WAIT4GUEST", "QASPROCESSING"]
        self.tm = tm(speech=True, pytoolkit=True)
        self.tm.initialize_node(self.task_name)
        self.qas = self.cargar_preguntas()
        '''
        Los estados son:
        - PREGUNTADOS: Estado inicial del juego.
        - INIT: Inicializacion del robot y explicacion del juego.
        - WAIT4GUEST: Espera a que un invitado diga "Empezar juego".
        - ASK: Se hace una pregunta y se espera la respuesta del usuario.
        - CHECKANSWER: Se verifica si la respuesta es correcta.
        - SHOWANSWER: Se muestra la respuesta correcta y el puntaje actual.
        '''
        # TODO: Implementar interfaz grafica para mostrar preguntas y respuestas
        # TODO: Implementar funciones varias que permitan el movimiento y expresion del robot

        # Se definen las transiciones entre estados
        transitions = [
            {"trigger": "start", "source": "TRIVIA", "dest": "INIT"},
            {"trigger": "begin", "source": "INIT", "dest": "WAIT4GUEST"},
            {"trigger": "new_game_start", "source": "WAIT4GUEST", "dest": "PREGUNTADOS"},
            {"trigger": "restart", "source": "PREGUNTADOS", "dest": "WAIT4GUEST"},
        ]
        self.machine = Machine(model=self, states=states, transitions=transitions, initial='TRIVIA')
        rospy_check = threading.Thread(target=self.check_rospy)
        rospy_check.start()
        
    def on_enter_INIT(self):
        print(self.consoleFormatter.format("Inicializacion del task: "+self.task_name, "HEADER"))
        self.tm.initialize_pepper()
        self.enable_breathing_service()
        self.tm.hot_word(["jugar"], thresholds=[0.4],language="Spanish")
        subscriber = rospy.Subscriber("/pytoolkit/ALSpeechRecognition/status",speech_recognition_status_msg,self.callback_hot_word)
        self.tm.talk("Hola, soy Pepper. Vamos a jugar a Preguntados. Te deseo suerte.", language="Spanish")
        self.tm.talk("El juego consiste en que yo te hare preguntas de diferentes categorias y tu deberas responderlas.", language="Spanish")
        self.tm.talk("Tienes que decir la respuesta que consideres correcta exactamente igual que como yo la diga", language="Spanish")
        self.begin()
    
    def on_enter_WAIT4GUEST(self):
        print(self.consoleFormatter.format("Esperando a un invitado...", "HEADER"))
        self.tm.talk("Estoy esperando a que un invitado me diga: Empezar juego, para comenzar una nueva partida.", language="Spanish")
        speech2text_thread = threading.Thread(target=self.speech2text_function)
        speech2text_thread.start()    
            
        while self.answer == "":
            rospy.sleep(0.1)
        speech2text_thread.join(20)
        while self.new_game==False:
            rospy.sleep(0.1)
        self.tm.talk("Perfecto, vamos a empezar una nueva partida.", language="Spanish")
        self.new_game_start()
    
    def on_enter_PREGUNTADOS(self):
        print(self.consoleFormatter.format("Comenzando el juego de Preguntados", "HEADER"))
        self.tm.talk("Cuantas rondas de preguntas quieres jugar?", language="Spanish")
        rounds_text = self.tm.speech2text_srv()
        try:
            self.total_rounds = int(rounds_text)
        except ValueError:
            self.total_rounds = 5
            self.tm.talk("No he entendido el numero, jugaremos 5 rondas por defecto.", language="Spanish")
        # Inicializar variables del juego
        self.score = 0
        self.question_number = 1
        self.is_correct = False
        # Empezar el juego
        self.tm.talk("Muy bien, ya estamos listos para comenzar. Te hare la primera pregunta.", language="Spanish")
        # Bucle principal del juego, se hacen las preguntas y se verifica la respuesta, esto basado en el numero de rondas provisto
        for _ in range(self.total_rounds):
            self.QASPROCESSING(self)
            if self.is_correct:
                self.tm.talk("Correcto! Has respondido bien.", language="Spanish")
                self.score += 1
            else:
                self.tm.talk(f"Incorrecto. La respuesta correcta era: {self.current_answers['correct']}.", language="Spanish")
            self.tm.talk(f"Tu puntaje actual es: {self.score}.", language="Spanish")
        self.tm.talk(f"El juego ha terminado. Tu puntaje final es: {self.score} de {self.total_rounds}. Gracias por jugar!", language="Spanish")
        # Se vuelve al estado WAIT4GUEST para esperar un nuevo jugador 
        self.restart()
        
    def QASPROCESSING(self):
        '''Se hace la pregunta y se espera la respuesta del usuario'''
        # Se elige una tematica y una pregunta aleatoriamente
        preguntas_respuestas = self.qas
        tematica = random.choice(list(preguntas_respuestas.keys()))
        # Seleccionar una pregunta y sus respuestas, en pantalla debe verse la tematica, pregunta y opciones
        pregunta, respuestas = random.choice(preguntas_respuestas[tematica])
        # Guardar la pregunta y respuestas actuales
        self.current_question = pregunta
        self.current_answers = respuestas
        # Preguntar mediante talk y ofrecer las opciones
        self.tm.talk(
        f"Pregunta {self.question_number}: {pregunta}. "
        f"Las opciones son: {', '.join(respuestas['options'])}."
        "Por favor, responde con la opcion correcta.", language="Spanish")
        # Esperar la respuesta del usuario mediante speech2text
        self.answer=""
            
        speech2text_thread = threading.Thread(target=self.speech2text_function)
        speech2text_thread.start()    
            
        while self.answer == "":
            rospy.sleep(0.1)
        speech2text_thread.join(20)
        while self.listo_respuesta==False:
            rospy.sleep(0.1)    
        # Obtener la respuesta del usuario
        respuesta = self.tm.speech2text_srv(seconds=5, lang="esp")
        if respuesta is None:
            rospy.sleep(0.1)
        # Validar que la respuesta este entre las opciones, en la pantalla muestra si es correcto o no
        while respuesta not in self.current_answers["options"]:
            respuesta = self.tm.speech2text_srv(seconds=5, lang="esp")
            self.tm.talk("No he podido entender tu respuesta. Por favor, intenta de nuevo.", language="Spanish")
            self.answer = None
        # Almacenar la respuesta del usuario
        if respuesta in self.current_answers["options"]:
            print(self.consoleFormatter.format("He escuchado: "+respuesta, "OKGREEN"))
            self.answer = respuesta
        # Se verifica si la respuesta es correcta, en patalla se muestra el puntaje actual
        self.is_correct = (self.answer == self.current_answers["correct"])
        ''''Se muestra el puntaje en pantalla, falta implementacion de parte de interface para esto'''
        self.question_number += 1
        
    def callback_hot_word(self,data):
        # Callback para el hotword "jugar" y "respuesta"
        word = data.status
        print(word, " listened")
        if word=="jugar":
            self.new_game = True
        if word == "respuesta":
            self.listo_respuesta = False
            
    
    
    """Funciones auxiliares"""
    
    def cargar_preguntas(self):
        with open(os.path.join(os.path.dirname(__file__), "preguntas_database.json"), "r", encoding="utf-8") as f:
            data = json.load(f)
        qas = {}
        for item in data:
            categoria = item["categoria"]
            pregunta = item["pregunta"]
            opciones = item["opciones"]
            correcta = item["correcta"]
            if "Entretenimiento" in categoria:
                categoria = "Entretenimiento"
            elif "Ciencia" in categoria:
                categoria = "Ciencia"
            elif "Mitología" in categoria:
                categoria = "Historia"
            elif "general" in categoria:
                categoria = "Conocimiento General"
            if categoria not in qas:
                qas[categoria] = []
            qas[categoria].append((pregunta, {"options": opciones, "correct": correcta}))
        print("Preguntas cargadas y clasificadas en:")
        for cat, preguntas in qas.items():
            print(f"{cat}: {len(preguntas)} preguntas")
        return qas
    
    def check_rospy(self):
       #Termina todos los procesos al cerrar el nodo
       while not rospy.is_shutdown():
           time.sleep(0.1)
       print(self.consoleFormatter.format("Shutting down", "FAIL"))
       os._exit(os.EX_OK)

    def run(self):
        while not rospy.is_shutdown():
            self.start()
    
if __name__ == "__main__":
   sm = Preguntados()
   sm.run()
   rospy.spin()