#!/usr/bin/env python3
from transitions import Machine
from task_module import Task_module as tm
import ConsoleFormatter
import time
import threading
import rospy
import os
import re
import math
import ast
from code_generation import ls_generate as gen
from code_generation import generate_utils 
from code_generation.database.models import Model
class EGPSR(object):

    def __init__(self):
        self.gen = gen.LongStringGenerator()

        self.consoleFormatter=ConsoleFormatter.ConsoleFormatter()
        # Definir los estados posibles del semáforo
        self.task_name = "EGPSR"
        states = ['INIT', 'SEARCH4GUEST', 'EGPSR']
        self.tm = tm(perception = True,speech=True,manipulation=True, navigation=True, pytoolkit=True)
        self.tm.initialize_node(self.task_name)
        # Definir las transiciones permitidas entre los estados
        transitions = [
            {'trigger': 'start', 'source': 'EGPSR', 'dest': 'INIT'},
            {'trigger': 'beggining', 'source': 'INIT', 'dest': 'SEARCH4GUEST'},
            {'trigger': 'person_raised_hand', 'source': 'SEARCH4GUEST', 'dest': 'EGPSR'},
            {'trigger': 'again', 'source': 'SEARCH4GUEST', 'dest': 'SEARCH4GUEST'},
            {'trigger': 'GPSR_done', 'source': 'EGPSR', 'dest': 'SEARCH4GUEST'}
        ]
        
        # State machine creation
        self.machine = Machine(model=self, states=states, transitions=transitions, initial='EGPSR')
        rospy_check = threading.Thread(target=self.check_rospy)
        rospy_check.start()
        
        ############################# GLOBAL VARIABLES #############################
        self.init_place = "house_door"
        self.places_names = ["kitchen","living_room","dining","bedroom"] #TODO: Change if there are more places
        self.head_angles = [0,60,-60]
        self.task_counter = 0
        
        self.angles_to_check = self.head_angles
        self.last_checked_angle = None
        
        self.last_angle_checked = 0

    def on_enter_INIT(self):
        self.tm.initialize_pepper()
        self.tm.turn_camera("bottom_camera","custom",1,15)
        # CAMBIAR EN LA COMPETENCIA, EL ROBOT NO INICIA EN GPSR_LOCATION DEBE NAVEGAR ALLA TODO
        self.tm.current_place = "house_door"
        self.tm.set_current_place(self.init_place)
        self.tm.turn_camera("depth_camera","custom",1,15)
        self.tm.toggle_filter_by_distance(True,2,["person"])
        print(self.consoleFormatter.format("Inicializacion del task: "+self.task_name, "HEADER"))
        self.tm.talk("I am going to do the  "+ self.task_name + " task","English")
        self.beggining()
        
    def on_enter_EGPSR(self):
        print(self.consoleFormatter.format("EGPSR", "HEADER"))
        self.tm.look_for_object("person")
        self.tm.talk("Hello! please tell me what do you want me to do. I will try to execute the task you give me. Please talk loud and say the task once. You can talk to me when my eyes are blue: ","English")
        task = self.tm.speech2text_srv(0)
        self.generated_code = ""

        # Thread to call the method that generates code according to a task request
        code_gen_thread = threading.Thread(target=self.code_gen_t,args=[task])
        code_gen_thread.start()

        self.tm.talk(f"Your command is {task}","English", wait=True)
        self.tm.talk("If that is correct, please touch my head. If not, please wait until my eyes are blue again","English", wait=False)
        
        correct = self.tm.wait_for_head_touch(message="", message_interval=100, timeout=13)
        
        while not correct:
            self.tm.talk("I am sorry, may you please repeat your command?","English", wait=True)
            task = self.tm.speech2text_srv(0)

            # Thread to call the method that generates code according to a task request after having failed in the first attempt
            code_gen_thread = threading.Thread(target=self.code_gen_t,args=[task])
            code_gen_thread.start()

            self.tm.talk(f"Your command is {task}. If that is correct, please touch my head","English", wait=True)
            correct = self.tm.wait_for_head_touch(message="", message_interval=13, timeout=13)
        
        self.tm.talk("I am processing your request")

        while self.generated_code == "":
            rospy.sleep(0.1)
        
        self.tm.talk("I am now analyzing the way to execute the task you gave me","English", wait=False)
        
        # Execution of the generated code if it is possible
        if self.generated_code != "impossible":
            exec(self.generated_code)

        print(f"Task: {task}")
        self.GPSR_done()
        

    def on_enter_SEARCH4GUEST(self):
        print(self.consoleFormatter.format("SEARCH4GUEST", "HEADER"))
        self.tm.go_to_pose("default_head")
        self.tm.go_to_pose("standard")
        self.current_place = self.places_names[0]
        
        self.tm.talk(f"I am going to search a guest in the {self.current_place}","English", False)
        self.tm.go_to_place(self.current_place)
        
        self.tm.talk("I am now checking the whole room to see if there is any person who may need my help.","English", wait=False)
        
        # Stablishing the angles to check in the room
        if self.last_checked_angle is None:
            self.angles_to_check = self.head_angles
            
        # Checking the angles in the room
        for angle in self.angles_to_check:
            
            print("\nCurrent Angle:",angle,"\n")
            self.tm.set_angles_srv(["HeadYaw","HeadPitch"],[math.radians(angle), -0.1],0.1)
            
            if angle==0:
                rospy.sleep(3)
                
            elif angle==-60:
                rospy.sleep(3)
                
            elif angle==60:
                rospy.sleep(5)
                
            self.tm.labels = dict()
            rospy.sleep(2)
            persons = self.tm.labels.get("person", [])

            # Updating the last checked angle to return to it after completing a task
            self.last_angle_checked = angle
            
            # Erasing the angle to check when only ONE person was detected
            if len(persons)<=1:
                self.angles_to_check.remove(angle)
                
            # Pepper will not check another room until all the people seen have been checked
            for person in persons:
                
                # Centering the robot camera with the person
                self.tm.talk("I will check if you are raising your hand to request my help","English", wait=False)
                self.tm.center_head_with_label(person)

                self.tm.talk("Give me one moment please!","English", wait=False)
                person_response = self.tm.img_description(
                        "Observe the person in front of you and respond only with True if the person is raising their hand. Otherwise, respond only with False. Here is an example output: False",
                        "front_camera")
                
                is_raisin = True if person_response["message"] == "True" else False

                self.tm.set_angles_srv(["HeadYaw","HeadPitch"],[math.radians(angle), -0.1],0.1)
                
                if is_raisin:
                    # Going to the EGPSR state to ask the person for the task
                    self.person_raised_hand()
        
        self.places_names.pop(0)
        self.tm.talk("I have finished checking this room!","English", wait=False)
        self.last_checked_angle = None

        # Returning to this same state to check another room
        self.again()

        
    # Thread to generate code
    def code_gen_t(self, task):
        
        print(self.consoleFormatter.format("GEN CODE THREAD", "HEADER"))
        
        contador = 0
        code = ""

        while contador<1:
            code = self.gen.generate_code(task, Model.GPT4).replace("`","").replace("python","")
            pattern = r'self\.tm\.go_to_place\((.*?)\)'
            code = re.sub(pattern, r'self.tm.go_to_place(\1, lower_arms=False)', code)
            print(code)

            if "I am sorry but I cannot complete this task" not in code:
                print("\nIt is possible to execute the request")

                if self.is_valid_syntax(code):
                    self.generated_code = code
                    self.task_counter += 1
                    contador = 5
            contador += 1

        if contador==1:
            self.generated_code = "impossible"
            self.tm.talk("I cannot the following task: " + task,"English")
        
    def check_rospy(self):
        while not rospy.is_shutdown():
            time.sleep(0.1)
        print(self.consoleFormatter.format("Shutting down", "FAIL"))
        os._exit(os.EX_OK)

    def run(self):
        while not rospy.is_shutdown():
            self.start()
    
    def is_valid_syntax(self, code):
        try:
            ast.parse(code)
            return True
        except SyntaxError:
            return False
    
# Crear una instancia de la maquina de estados
if __name__ == "__main__":
    sm = EGPSR()
    sm.run()
    rospy.spin()