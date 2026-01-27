#!/usr/bin/env python3
from transitions import Machine
from task_module import Task_module as tm
import ConsoleFormatter
import time
import threading
import rospy
import os

class MERCADITO(object):
   def __init__(self, base_url):
       self.consoleFormatter = ConsoleFormatter.ConsoleFormatter()
       self.task_name = "CONVERSACION"
       self.base_url = base_url 
       self.answer = ""
       
       # Initialize Robot
       self.tm = tm(speech=True, pytoolkit=True)
       try:
           self.tm.initialize_node(self.task_name)
       except:
           pass

       # --- STATES ---
       states = ['IDLE', 'INIT', 'HABLAR', 'FINISH']

       # --- TRANSITIONS ---
       transitions = [
           {'trigger': 'start_loop', 'source': 'IDLE',   'dest': 'INIT'},
           {'trigger': 'next_step',  'source': 'INIT',   'dest': 'HABLAR'},
           {'trigger': 'finalize',   'source': 'HABLAR', 'dest': 'FINISH'},
           {'trigger': 'loop_back',  'source': 'FINISH', 'dest': 'INIT'},
       ]

       self.machine = Machine(model=self, states=states, transitions=transitions, initial='IDLE')
       
       # Start background logic thread
       self.stop_thread = False
       threading.Thread(target=self.run_logic_loop).start()

   def run_logic_loop(self):
       time.sleep(5) # Wait for FastAPI to start
       self.start_loop() # Go to INIT

   # --- STATE ACTIONS ---

   def on_enter_INIT(self):
       print("--- STATE: INIT (Loading index.html) ---")
       
       # 1. Load INDEX.HTML
       self.tm.show_web_view(f"{self.base_url}/")
       
       # 2. Logic
       self.tm.initialize_pepper()
       self.tm.talk(text="Hello, I am Nova. How are you?", language="English")
       
       # 3. Move to next state
       self.next_step()

   def on_enter_HABLAR(self):
       print("--- STATE: HABLAR (Listening...) ---")
       
       # 1. Listen first
       self.answer = self.tm.speech2text_srv(seconds=0, lang="eng")
       
       # 2. Load MENU.HTML with the answer as a parameter
       # We encode spaces as %20
       safe_answer = self.answer.replace(" ", "%20")
       self.tm.show_web_view(f"{self.base_url}/menu?text={safe_answer}")

       # 3. Repeat what was heard
       self.tm.talk(text=f"You said: {self.answer}", language="English")
       
       # 4. Move to next state
       self.finalize()

   def on_enter_FINISH(self):
       print("--- STATE: FINISH ---")
       
       self.tm.talk(text="Finish", language="English")
       time.sleep(2)
       
       # Loop back to start
       self.loop_back()