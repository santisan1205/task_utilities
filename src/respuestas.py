#!/usr/bin/env python3
from transitions import Machine
from task_module import Task_module as tm
import ConsoleFormatter
import time
import threading
import rospy
import os
import json
import random

class PREGUNTADOS(object):
    def __init__(self):
        self.consoleFormatter = ConsoleFormatter.ConsoleFormatter()
        self.task_name = "PREGUNTADOS"
        self.tm = tm(speech=True, pytoolkit=True)
        self.tm.initialize_node(self.task_name)

        with open(os.path.join(os.path.dirname(__file__), "preguntas.json"), "r", encoding="utf-8") as f:
            self.questions = json.load(f)
        self.current_question = None
        self.question_count = 0
        self.max_questions = 10

        self.shuffled_questions = random.sample(self.questions, self.max_questions)

        states = ['INICIO', 'EXPLICAR_JUEGO', 'HACER_PREGUNTA', 'ESCUCHAR_RESPUESTA', 'DAR_FEEDBACK', 'FINAL']
        transitions = [
            {'trigger': 'start', 'source': 'INICIO', 'dest': 'EXPLICAR_JUEGO'},
            {'trigger': 'explicado', 'source': 'EXPLICAR_JUEGO', 'dest': 'HACER_PREGUNTA'},
            {'trigger': 'preguntado', 'source': 'HACER_PREGUNTA', 'dest': 'ESCUCHAR_RESPUESTA'},
            {'trigger': 'respondido', 'source': 'ESCUCHAR_RESPUESTA', 'dest': 'DAR_FEEDBACK'},
            # loop si hay más preguntas, si no pasa a FINAL
            {'trigger': 'siguiente', 'source': 'DAR_FEEDBACK', 'dest': 'HACER_PREGUNTA'},
            {'trigger': 'terminar', 'source': 'DAR_FEEDBACK', 'dest': 'FINAL'},
        ]
        self.correct_answers = 0
        # Make the whole body breath
        self.tm.enable_breathing_service(chain="All",state=True)
        # Except the head, so tracking works
        self.tm.enable_breathing_service(chain="Head",state=False)
        self.tm.motion_tools_service()
        self.tm.start_tracker_proxy()
        self.tm.show_words_proxy()
        rospy.sleep(5)

        self.machine = Machine(model=self, states=states, transitions=transitions, initial='INICIO')
        rospy_check = threading.Thread(target=self.check_rospy)
        rospy_check.start()

    def on_enter_EXPLICAR_JUEGO(self):
        print(self.consoleFormatter.format("EXPLICAR_JUEGO", "HEADER"))
        self.tm.talk(text="¡Vamos a jugar Preguntados! Te haré diez preguntas. Intenta responderlas correctamente. ¿Listo?", language="Spanish",animated=True)
        self.explicado()

    def on_enter_HACER_PREGUNTA(self):
        print(self.consoleFormatter.format("HACER_PREGUNTA", "HEADER"))
        self.current_question = self.shuffled_questions[self.question_count]
        self.tm.talk(text=f"Pregunta {self.question_count + 1}: {self.current_question['q']}", language="Spanish",animated=True)
        self.preguntado()

    def on_enter_ESCUCHAR_RESPUESTA(self):
        print(self.consoleFormatter.format("ESCUCHAR_RESPUESTA", "HEADER"))
        response = self.tm.speech2text_srv(seconds=0, lang="esp")
        self.user_answer = response.lower().strip()
        self.respondido()

    def on_enter_DAR_FEEDBACK(self):
        print(self.consoleFormatter.format("DAR_FEEDBACK", "HEADER"))
        if "none" in self.user_answer:
            self.tm.talk(text="Disculpa, no te entendi. Repite tu respuesta", language="Spanish",animated=False)
            response = self.tm.speech2text_srv(seconds=0, lang="esp")
            self.user_answer = response.lower().strip()
        if self.current_question["a"] in self.user_answer:
            self.tm.play_animation(animation_name="Emotions/Positive/Winner_2")
            self.tm.talk(text="¡Correcto! Muy bien hecho.", language="Spanish",animated=False,wait=True)
            rospy.sleep(2)
            self.correct_answers += 1
        else:
            self.tm.talk(text=f"¡Uy! La respuesta correcta era {self.current_question['a']}.", language="Spanish",animated=True)

        self.question_count += 1
        if self.question_count < self.max_questions:
            self.siguiente()
        else:
            self.terminar()

    def on_enter_FINAL(self):
        print(self.consoleFormatter.format("FINAL", "HEADER"))
        self.tm.talk(
            text=f"¡Eso fue todo! Respondiste correctamente {self.correct_answers} de {self.max_questions} preguntas. "
                "Gracias por jugar Preguntados conmigo. ¡Hasta la próxima!",
            language="Spanish",animated=True)
        os._exit(os.EX_OK)

    def check_rospy(self):
        while not rospy.is_shutdown():
            time.sleep(0.1)
        print(self.consoleFormatter.format("Cerrando el nodo", "FAIL"))
        os._exit(os.EX_OK)

    def run(self):
        while not rospy.is_shutdown():
            self.start()

if __name__ == "__main__":
    sm = PREGUNTADOS()
    sm.run()
    rospy.spin()
