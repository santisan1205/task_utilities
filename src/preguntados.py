from transitions import Machine
from task_module import task_module as tm
import threading
import time
import rospy
import os
import ConsoleFormatter
import random
import json
class Preguntados(object):
    def __init__(self) -> None:
        self.consoleFormatter=ConsoleFormatter.ConsoleFormatter()
        self.task_name = "PREGUNTADOS"
        self.hearing = True

        states =  ["PREGUNTADOS", "INIT", "WAIT4GUEST", "ASK", "CHECKANSWER", "SHOWANSWER"]
        self.tm = tm(speech=True, pytoolkit=True)
        self.tm.initialize_node(self.task_name)
        self.qas = cargar_preguntas()
        '''
        Los estados son:
        - PREGUNTADOS: Estado inicial del juego.
        - INIT: Inicializacion del robot y explicacion del juego.
        - WAIT4GUEST: Espera a que un invitado diga "Empezar juego".
        - ASK: Se hace una pregunta y se espera la respuesta del usuario.
        - CHECKANSWER: Se verifica si la respuesta es correcta.
        - SHOWANSWER: Se muestra la respuesta correcta y el puntaje actual.
        '''
        # TODO: Cargar preguntas y respuestas desde un archivo JSON o base de datos
        # TODO: Implementar interfaz grafica para mostrar preguntas y respuestas
        # TODO: Implementar funciones varias que permitan el movimiento y expresion del robot
        # TODO: Deteccion de numeros mediante entrada de voz
        # TODO: Manejo de errores y excepciones
        # TODO: Documentacion, depuracion y legibilidad del codigo
        
        # Se definen las transiciones entre estados
        transitions = [
            {"trigger": "start", "source": "PREGUNTADOS", "dest": "INIT"},
            {"trigger": "begin", "source": "INIT", "dest": "WAIT4GUEST"},
            {"trigger": "new_game_start", "source": "WAIT4GUEST", "dest": "PREGUNTADOS"},
            {"trigger": "finish", "source": "PREGUNTADOS", "dest": "WAIT4GUEST"},
            {"trigger": "ask_question", "source": "PREGUNTADOS", "dest": "ASK"},
            {"trigger": "check_answer", "source": "ASK", "dest": "CHECKANSWER"},
            {"trigger": "show_answer", "source": "CHECKANSWER", "dest": "SHOWANSWER"},
            {"trigger": "restart", "source": "PREGUNTADOS", "dest": "WAIT4GUEST"},
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
                    self.tm.talk("Perfecto, vamos a empezar una nueva partida.")
                    self.new_game_start()
    
    
    def on_enter_ask_question(self):
        '''Se hace la pregunta y se espera la respuesta del usuario'''
        # Se elige una tematica y una pregunta aleatoriamente
        preguntas_respuestas = self.qas
        tematica = random.choice(list(preguntas_respuestas.keys()))
        # Seleccionar una pregunta y sus respuestas
        pregunta, respuestas = random.choice(preguntas_respuestas[tematica])
        # Guardar la pregunta y respuestas actuales
        self.current_question = pregunta
        self.current_answers = respuestas
        # Preguntar mediante talk y ofrecer las opciones
        self.tm.talk(f"Pregunta {self.question_number}: {pregunta}. Las opciones son: {', '.join(respuestas)}. Por favor, dime tu respuesta.")
        respuesta = self.tm.speech2text_srv()
        # Validar que la respuesta este entre las opciones
        if respuesta in self.current_answers:
            print(self.consoleFormatter.format("He escuchado: "+respuesta, "OKGREEN"))
            self.answer = respuesta
            self.check_answer()
        else:
            self.tm.talk("No he podido entender tu respuesta. Por favor, intenta de nuevo.")
            self.ask_question()
    
    def on_enter_check_answer(self):
        # Se verifica si la respuesta es correcta
        if self.answer == self.current_answers["correct"]:
            self.is_correct = True
        else:
            self.is_correct = False
        self.show_answer()
    
    def on_enter_show_answer(self):
        ''''Se muestra el puntaje en pantalla, falta implementacion de parte de interface para esto'''
        self.question_number += 1
        
    def on_enter_PREGUNTADOS(self):
        print(self.consoleFormatter.format("Comenzando el juego de Preguntados", "HEADER"))
        self.tm.talk("Cuantas rondas de preguntas quieres jugar?")
        rounds_text = self.tm.speech2text_srv()
        """Nota: El numero debe traducirse del lenguaje al valor numerico. Ejemplo: "cinco" -> 5
        Se requiere una implementacion que haga esta traduccion."""
        try:
            self.total_rounds = int(rounds_text)
        except ValueError:
            self.total_rounds = 5
            self.tm.talk("No he entendido el numero, jugaremos 5 rondas por defecto.")
        # Inicializar variables del juego
        self.score = 0
        self.question_number = 1
        self.is_correct = False
        # Empezar el juego
        self.tm.talk("Muy bien, ya estamos listos para comenzar. Te hare la primera pregunta.")
        # Bucle principal del juego, se hacen las preguntas y se verifica la respuesta, esto basado en el numero de rondas provisto
        for _ in range(self.total_rounds):
            self.ask_question()
            if self.is_correct:
                self.tm.talk("Correcto! Has respondido bien.")
                self.score += 1
            else:
                self.tm.talk(f"Incorrecto. La respuesta correcta era: {self.current_answers['correct']}.")
            self.tm.talk(f"Tu puntaje actual es: {self.score}.")
        self.tm.talk(f"El juego ha terminado. Tu puntaje final es: {self.score} de {self.total_rounds}. Gracias por jugar!")
        # Se vuelve al estado WAIT4GUEST para esperar un nuevo jugador 
        self.restart()
    
    """Funciones auxiliares"""
    
    def cargar_preguntas():
        # Falta definir si se usa un JSON custom o OpenTDB
        pass