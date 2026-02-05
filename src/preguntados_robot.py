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
    
    def __init__(self, base_url) -> None:
        """Clase que representa el juego de Preguntados implementado como una máquina de estados.
        Atributos:
            consoleFormatter (ConsoleFormatter): Formateador de texto para consola.
            task_name (str): Nombre del task.
            tm (task_module): Módulo de tareas para interacción con el robot.
            qas (dict): Diccionario con preguntas y respuestas.
            url_base (str): URL base para la interfaz web del robot.
        """
        self.consoleFormatter=ConsoleFormatter.ConsoleFormatter()
        self.task_name = "PREGUNTADOS"
        self.hearing = True
        states =  ["PREGUNTADOS", "INIT", "WAIT4GUEST"]
        self.tm = tm(speech=True, pytoolkit=True)
        self.tm.initialize_node(self.task_name)
        self.qas = self.cargar_preguntas()
        self.base_url = base_url
        '''
        Los estados son:
        - PREGUNTADOS: Estado inicial del juego.
        - INIT: Inicializacion del robot y explicacion del juego.
        - WAIT4GUEST: Espera a que un invitado diga "Empezar juego".
        - ASK: Se hace una pregunta y se espera la respuesta del usuario.
        - CHECKANSWER: Se verifica si la respuesta es correcta.
        - SHOWANSWER: Se muestra la respuesta correcta y el puntaje actual.
        '''
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
        self.tm.show_webview(self.base_url + "/")
        self.current_category = ""
        self.ui_icon = ""
        self.ui_message = ""
        self.tm.enable_breathing_service()
        self.new_game = False
        self.answer = ""
        self.listo_respuesta = False
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
        while self.new_game==False:
            rospy.sleep(0.1)
        speech2text_thread.join(20)
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
        self.tm.show_webview(self.base_url + "/")

        # Empezar el juego
        self.tm.talk("Muy bien, ya estamos listos para comenzar. Te hare la primera pregunta.", language="Spanish")
        # Bucle principal del juego, se hacen las preguntas y se verifica la respuesta, esto basado en el numero de rondas provisto
        for _ in range(self.total_rounds):
            self.QASPROCESSING()
            # Valida la respuesta, actualiza el puntaje y manda la informacion a la API para mostrar en la tablet
            if self.is_correct:
                self.score += 1
                self.ui_icon = "/static/correcto.png"
                self.ui_message = "¡Respuesta correcta!"
                self.tm.talk("Correcto! Has respondido bien.", language="Spanish")
            else:
                self.ui_icon = "/static/incorrecto.png"
                self.ui_message = f"Incorrecto. La respuesta correcta era: {self.current_answers['correct']}"
                self.tm.talk(self.ui_message, language="Spanish")
            self.tm.show_webview(self.base_url + "/resultado")
            rospy.sleep(2)
            self.tm.talk(f"Tu puntaje actual es: {self.score}.", language="Spanish")
        self.ui_icon = "/static/trofeo.png"
        self.ui_message = "Juego terminado"
        self.tm.show_webview(self.base_url + "/resultado")
        rospy.sleep(3)
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
        self.current_category = tematica
        self.tm.show_webview(self.base_url + "/pregunta")
        # Preguntar mediante talk y ofrecer las opciones
        self.tm.talk(
        f"Pregunta {self.question_number}: {pregunta}. "
        f"Las opciones son: {', '.join(respuestas['options'])}."
        "Por favor, responde con la opcion correcta.", language="Spanish")
        self.answer = ""
        self.listo_respuesta = False
        # Esperar la respuesta del usuario mediante speech2text
        speech2text_thread = threading.Thread(target=self.speech2text_function)
        speech2text_thread.start()    
        # Pone una bandera, el usuario dice la opcion y finaliza con la palabra "respuesta"
        # Ejemplo importante: "Paris respuesta"
        while self.listo_respuesta==False:
            rospy.sleep(0.1)
        speech2text_thread.join(20)
        # Obtener la respuesta del usuario
        if self.answer is None:
            rospy.sleep(0.1)
        # Almacenar la respuesta del usuario
        opciones_validas = [op.lower().strip() for op in self.current_answers["options"]]
        respuesta_usuario = (self.answer or "").lower().strip()
        # Validar que la respuesta este entre las opciones, en la pantalla muestra si es correcto o no
        while respuesta_usuario not in opciones_validas:
            self.tm.talk("Tu respuesta no se encuentra entre las opciones, intenta de nuevo.", language="Spanish")

            self.answer = ""
            self.listo_respuesta = False
            
            speech2text_thread = threading.Thread(target=self.speech2text_function)
            speech2text_thread.start()
            # EL USUARIO DEBE DECIR LA PALABRA "RESPUESTA" PARA QUE SE PROCEDA A LEER LA RESPUESTA IMPORTANTE!!!
            while not self.listo_respuesta:
                rospy.sleep(0.1)

            speech2text_thread.join(20)

            respuesta_usuario = (self.answer or "").lower().strip()
        print(self.consoleFormatter.format("He escuchado: " + respuesta_usuario, "OKGREEN"))
        respuesta_correcta = self.current_answers["correct"].lower().strip()
        # Se verifica si la respuesta es correcta
        self.is_correct = (respuesta_usuario == respuesta_correcta)
        self.question_number += 1
        
    def callback_hot_word(self,data):
        # Callback para el hotword "jugar" y "respuesta"
        word = data.status
        print(word, " listened")
        if word=="jugar":
            self.new_game = True
        if word == "respuesta":
            self.listo_respuesta = True
            
    def speech2text_function(self):
        while self.answer=="":
            temporal = self.tm.speech2text_srv(seconds=0,lang="esp")
            if not ("None" in temporal):
                self.answer=temporal
            else:
                self.tm.talk(text="Disculpa, no te entendi. Por favor repite tu respuesta ",language="Spanish", wait=True, animated=False)
    
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