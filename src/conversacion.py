#!/usr/bin/env python3
from transitions import Machine
from task_module import Task_module as tm
import ConsoleFormatter
import time
import threading
import rospy
import os

class MERCADITO(object):
   def __init__(self):

       self.consoleFormatter=ConsoleFormatter.ConsoleFormatter()
       # Definir los estados posibles del semáforo
       #--------------------------------------------------------------------------INICIALIZACION TASK MODULE/ROBOT
       self.task_name = "CONVERSACION"
       self.tm = tm(speech=True, pytoolkit=True)
       self.tm.initialize_node(self.task_name)
       
       #--------------------------------------------------------------------------MAQUIMA DE ESTADOS
       
       states = ['CONVERSACION','INIT', 'HABLAR','FINISH','CONVERSACION_DONE']
       # Definir las transiciones permitidas entre los estados
       transitions = [
           {'trigger': 'start', 'source': 'CONVERSACION', 'dest': 'INIT'},
           {'trigger': 'beggining', 'source': 'INIT', 'dest': 'HABLAR'},
           {'trigger': 'hablar_ready', 'source': 'HABLAR', 'dest': 'FINISH'},
           {'trigger': 'finish', 'source': 'FINISH', 'dest': 'CONVERSACION_DONE'},
       ]

       # Crear la máquina de estados
       self.machine = Machine(model=self, states=states, transitions=transitions, initial='CONVERSACION')
       rospy_check = threading.Thread(target=self.check_rospy)
       rospy_check.start()

       ############################# STATES #############################

   def on_enter_INIT(self):
       print(self.consoleFormatter.format("INIT", "HEADER"))
       self.tm.initialize_pepper()
       self.tm.talk(text="Hello I'm nova. How are you?", language="English")
       self.beggining()

   def on_enter_HABLAR(self):
       print(self.consoleFormatter.format("HABLAR", "HEADER"))
       self.answer = self.tm.speech2text_srv(seconds=0,lang="eng")
       self.tm.talk(text = self.answer)
       self.hablar_ready()

   def on_enter_FININSH(self):
       print(self.consoleFormatter.format("FINISH", "HEADER"))
       self.finish()

   def on_enter_CONVERSACION_DONE(self):
       print(self.consoleFormatter.format("CONVERSACION_DONE", "HEADER"))
       
       os._exit(os.EX_OK)
   
   def check_rospy(self):
       #Termina todos los procesos al cerrar el nodo
       while not rospy.is_shutdown():
           time.sleep(0.1)
       print(self.consoleFormatter.format("Shutting down", "FAIL"))
       os._exit(os.EX_OK)

   def run(self):
       while not rospy.is_shutdown():
           self.start()
   
# Crear una instancia de la maquina de estados
if __name__ == "__main__":
   sm = MERCADITO()
   sm.run()
   rospy.spin()