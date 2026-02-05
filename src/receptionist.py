#!/usr/bin/env python3

# --------------------------- GENERAL LIBRARIES IMPORTS ------------------------------
from transitions import Machine
from task_module import Task_module as tm
import ConsoleFormatter
import threading
import rospy
import math
import time
import random
import os

# --------------------------- RECEPTIONIST TASK CLASS  ------------------------------
class RECEPTIONIST(object):
    
    # --------------------------- Class Constructor ------------------------------
    def __init__(self):

        self.consoleFormatter=ConsoleFormatter.ConsoleFormatter()
        
        # Task name
        self.task_name = "receptionist"
        
        # Possible states for the states machine
        states = ['INIT', 'GO2DOOR', 'WAIT4GUEST', 'SAVE_GUEST', 'GO2LIVING','INTRODUCE_GUEST']
        
        # Initialization of the task module
        self.tm = tm(perception = True,speech=True, navigation=True, pytoolkit=True, manipulation=True)
        self.tm.initialize_node(self.task_name)
        
        # Definition of the permitted transitions between states
        transitions = [
            {'trigger': 'start', 'source': 'RECEPTIONIST', 'dest': 'INIT'},
            {'trigger': 'beggining', 'source': 'INIT', 'dest': 'GO2DOOR'},
            {'trigger': 'arrive_house_door', 'source': 'GO2DOOR', 'dest': 'WAIT4GUEST'},
            {'trigger': 'guest_found', 'source': 'WAIT4GUEST', 'dest': 'SAVE_GUEST'},
            {'trigger': 'guest_saved', 'source': 'SAVE_GUEST', 'dest': 'GO2LIVING'},
            {'trigger': 'arrive_living_room', 'source': 'GO2LIVING', 'dest': 'INTRODUCE_GUEST'},
            {'trigger': 'guest_introduced', 'source': 'INTRODUCE_GUEST', 'dest': 'GO2DOOR'},
        ]
        
        # Creation of the states machine
        self.machine = Machine(model=self, states=states, transitions=transitions, initial='RECEPTIONIST')

        # Thread to check if rospy is running
        rospy_check = threading.Thread(target=self.check_rospy)
        rospy_check.start()
        

        # --------------------------- Task Parameters ------------------------------
        
        # The angles of the chairs the robot must check to introduce the people
        self.chair_angles = [-10,10,40]
        self.chair_distances = [1.9,1.9,2.1]
        

        self.host = {
            "name": "charlie",
            "age": self.categorize_age(21),
            "drink": "Milk",
            "gender": "Man",
            "pronoun": "he",
        }
        
        self.first_guest = {
            "hair": "",
            "clothes": "",
            "age": "",
            "gender": "",
        }
        
        self.second_guest = {}
        # TODO If room is big, use 2, else use 1
        self.resolution = 2
        self.saved_face = False
        self.host_saved = False
        self.guest_1_saved = False
        self.waiting_host = True
        self.guests_count = 1
        # Where the robot must introduce the guests
        self.seating_place = "living_room"
        self.initial_place = "init_stickler"
        self.greeting_place = "guests_door"
        self.last_place = self.initial_place
        

    # --------------------------- FIRST STATE: INIT ------------------------------
    def on_enter_INIT(self):
        
        self.tm.initialize_pepper()
        if self.resolution == 2:
            self.tm.turn_camera("front_camera","custom",2,10)
            self.tm.turn_camera("depth_camera","custom",1,10)
        else:
            self.tm.turn_camera("depth_camera","custom",1,15)
        self.tm.remove_faces_data()
        self.tm.publish_filtered_image("face", "front_camera")
        self.tm.set_current_place(self.initial_place)
        
        self.tm.show_topic("/perception_utilities/filtered_image")
        
        print(self.consoleFormatter.format("Inicializacion del task: "+self.task_name, "HEADER"))
        
        self.tm.talk("Executing the " + self.task_name + " task","English", wait=True)
        
        # Moving to the LOOK4PERSON state
        self.beggining()
        
    # --------------------------- SECOND STATE: GO TO THE HOUSE DOOR ------------------------------
    def on_enter_GO2DOOR(self):
        
        self.tm.talk("I'm navigating to the door to receive the coming guests!","English", wait=False)
        
        print(self.consoleFormatter.format("GO2DOOR", "HEADER"))
        
        self.tm.setRPosture_srv("stand")
        #TODO EL robot navega de otro lugar a la puerta
        self.tm.go_to_place(self.greeting_place)
        self.arrive_house_door()
        
    # --------------------------- THIRD STATE: WAIT FOR THE GUESTS ------------------------------
    def on_enter_WAIT4GUEST(self):
        print(self.consoleFormatter.format("WAIT4GUEST", "HEADER"))
        self.tm.go_to_pose("default_head")
        self.tm.setRPosture_srv("stand")
        rospy.sleep(1)
        self.tm.setMoveHead_srv.call("up")
        self.tm.talk("Waiting for guests!","English",wait=False)
        self.tm.look_for_object("person")
        self.tm.wait_for_object(-1)
        self.guest_found()

    # --------------------------- FOURTH STATE: SAVE GUEST ------------------------------
    def on_enter_SAVE_GUEST(self):
        
        print(self.consoleFormatter.format("SAVE_GUEST", "WARNING"))
        
        self.tm.set_angles_srv(["HeadYaw", "HeadPitch"],[0, -0.35],0.1)
        
        if self.guests_count == 1:
            self.tm.talk("Hi guest! please center your face in the circle in my tablet to register you!","English", wait=True)

            gpt_hair_thread = threading.Thread(target=self.gpt_hair_t)
            gpt_hair_thread.start()
            gpt_clothes_thread = threading.Thread(target=self.gpt_clothes_t)
            gpt_clothes_thread.start()
            save_face_thread = threading.Thread(target=self.save_face_t,args=["guest1"])
            save_face_thread.start()
            age_gender_thread = threading.Thread(target=self.age_gender_t)
            age_gender_thread.start()

            self.tm.talk("I will ask you a couple of questions while i get a good look at your face. Please answer them when my eyes turn blue!","English", wait=False)
            print("empieza sleep")
            rospy.sleep(10)
        else:
            self.tm.talk("Hi guest! I will ask you a couple of questions. Please answer them when my eyes turn blue!","English", wait=False)
            print("empieza sleep")
            rospy.sleep(8)
        print("acaba sleep")
        name = self.tm.q_a("What is your name?").lower()
        drink = self.tm.q_a("What is your favorite drink?")
        
        if self.guests_count == 1:
            self.first_guest["name"] = name
            self.first_guest["drink"] = drink
            
            while self.first_guest["age"]=="" or self.first_guest["gender"]=="":
                rospy.sleep(0.1)
                
        elif  self.guests_count == 2:
            self.second_guest["name"] = name
            self.second_guest["drink"] = drink
            
        self.tm.setRPosture_srv("stand")
        self.tm.talk("Thank you'!","English", wait=False)
        
        self.guest_saved()
        
    # --------------------------- FIFTH STATE: GO TO THE LIVING ROOM ------------------------------
    def on_enter_GO2LIVING(self):
        
        print(self.consoleFormatter.format("GO2LIVING", "HEADER"))
        
        self.tm.setRPosture_srv("stand")
        self.tm.show_topic("/perception_utilities/yolo_publisher")

        if self.guests_count == 1:
            self.tm.talk("Please follow me to the living room. I will introduce you to the other guests and give you a place to sit!","English", wait=False)

        elif  self.guests_count == 2:
            self.tm.talk(f"Please follow me to the living room! The host {self.host['name']} is waiting for you! Our guest {self.first_guest['name']} has already arrived. {self.first_guest['pronoun']} is a {self.first_guest['gender']} and {self.first_guest['pronoun']} is {self.first_guest['age']}. {self.first_guest['pronoun']} is wearing {self.first_guest['clothes']}, and has {self.first_guest['hair']} hair.", "English", wait=False)
        
        # Going to the living room
        self.tm.go_to_place(self.seating_place)

        self.arrive_living_room()
        
    
    # --------------------------- SIXTH STATE: INTRODUCE GUEST ------------------------------
    def on_enter_INTRODUCE_GUEST(self):
        
        print(self.consoleFormatter.format("INTRODUCE_GUEST", "WARNING"))
            
        if self.guests_count == 1:
            self.tm.talk(f'Hello everyone, this is {self.first_guest["name"]}, {self.first_guest["pronoun"]} is a {self.first_guest["gender"]}.  {self.first_guest["pronoun"]} is {self.first_guest["age"]}, and {self.first_guest["pronoun"]} likes to drink {self.first_guest["drink"]}.',"English", wait=False)
            current_guest = self.first_guest
        else:
            self.tm.talk(f'Hello everyone, this is {self.second_guest["name"]} and they like to drink {self.second_guest["drink"]}.',"English", wait=False)
            current_guest = self.second_guest
             
        found_guests = []   
        occupied_chairs = []
        
        for angle_number in range(len(self.chair_angles)):
            angle = self.chair_angles[angle_number]
            distance = self.chair_distances[angle_number]
            self.tm.toggle_filter_by_distance(True,distance,["person"])
            
            print("Angulo actual:",angle)

            self.tm.set_angles_srv(["HeadYaw","HeadPitch"],[math.radians(angle), -0.1], 0.12)
            
            sleep_time = 1 + ( 3.7 / 90 ) * abs(angle)
            
            rospy.sleep(sleep_time)
                
            self.tm.labels = dict()
            
            lower_bound = 0
            upper_bound = 1000
            
            if self.resolution == 1:
                lower_bound = 60
                upper_bound = 260
                
            elif self.resolution == 2:
                lower_bound = 180
                upper_bound = 460
                
            wait_time=2
            initial_time=time.time()
            found_person = False

            while (time.time() - initial_time<wait_time) and (not found_person):

                persons = self.tm.labels.get("person", [])
                
                for person in persons:
                    person_center = person[3]/2 +person[1]
                    if person_center> lower_bound and person_center < upper_bound:
                        found_person=True
                rospy.sleep(0.1)

            if not found_person:
                continue

            occupied_chairs.append(angle)
            
            self.tm.talk("Recognizing person!","English", wait=False)
            print("guests:",self.guests_count)
            if self.guests_count == 1:
                self.saved_face = False
                save_face_thread = threading.Thread(target=self.save_face_t,args=["charlie"])
                save_face_thread.start()
                self.tm.talk(f"Hey {self.first_guest['name']}! I introduce to you {self.host['name']}, {self.host['pronoun']} is a {self.host['gender']}. {self.host['pronoun']} is {self.host['age']}, and {self.host['pronoun']} likes to drink {self.host['drink']}.", "English", wait=False)
                while self.waiting_host:
                    rospy.sleep(0.1)
                print("host_saved")
                break
            else:
                guest_name = self.tm.recognize_face(3)
                print("guest:",guest_name)
                if not guest_name in found_guests:
                    found_guests.append(guest_name)
                if guest_name == "guest1":
                    self.tm.talk(f"Hey {self.second_guest['name']}! I introduce to you {self.first_guest['name']} is a {self.first_guest['gender']}. {self.first_guest['pronoun']} is {self.first_guest['age']}, and {self.first_guest['pronoun']} likes to drink {self.first_guest['drink']}.", "English", wait=True)
                elif guest_name == self.host["name"]:
                    self.tm.talk(f"Hey {self.second_guest['name']}! I introduce to you {self.host['name']} is a {self.host['gender']}. {self.host['pronoun']} is {self.host['age']}, and {self.host['pronoun']} likes to drink {self.host['drink']}.", "English", wait=True)
                else:
                    if not self.host_saved:
                        self.tm.talk(f"Hey {self.second_guest['name']}! I introduce to you {self.host['name']} is a {self.host['gender']}. {self.host['pronoun']} is {self.host['age']}, and {self.host['pronoun']} likes to drink {self.host['drink']}.", "English", wait=True)
                    elif not self.guest_1_saved:
                        self.tm.talk(f"{self.second_guest['name']}! I introduce to you {self.first_guest['name']} is a {self.first_guest['gender']}. {self.first_guest['pronoun']} is {self.first_guest['age']}, and {self.first_guest['pronoun']} likes to drink {self.first_guest['drink']}.", "English", wait=True)
                    else:
                        choice = random.choice([f"{self.second_guest['name']}! I introduce to you {self.first_guest['name']} is a {self.first_guest['gender']}. {self.first_guest['pronoun']} is {self.first_guest['age']}, and {self.first_guest['pronoun']} likes to drink {self.first_guest['drink']}.",f"Hey {self.second_guest['name']}! I introduce to you {self.host['name']} is a {self.host['gender']}. {self.host['pronoun']} is {self.host['age']}, and {self.host['pronoun']} likes to drink {self.host['drink']}."])
                        self.tm.talk(choice,"English",wait=True)
            if len(found_guests) == 2: 
                break

        self.tm.setRPosture_srv("stand")
        
        if len(occupied_chairs) == 0:
            self.tm.talk(f"I'm sorry i could not recognize anyone. May you please introduce yourself to {current_guest['name']}?", "English", wait=True)

        angles_to_sit_person = [angle for angle in self.chair_angles if angle not in occupied_chairs]
        angle_to_sit_person = random.choice(angles_to_sit_person)

        self.tm.set_angles_srv(["HeadYaw","HeadPitch"],[math.radians(angle_to_sit_person), -0.1],0.1)
        self.point_angle(angle_to_sit_person)
        sleep_time = 1 + ( 3.7 / 90 ) * abs(angle_to_sit_person)
    
        rospy.sleep(sleep_time)
        self.tm.talk("I see that chair is free. Please sit there.", "English", wait=True)
        self.tm.setRPosture_srv("stand") 
        
        if self.guests_count == 2:
            self.tm.talk("I have succesfully completed the Receptionist task! yeepee!", "English", wait=True)
            os._exit(os.EX_OK)
        self.guests_count +=1
        
                
        # Moving to the GO2NEXT state
        self.guest_introduced()
        
    # --------------------------- Complementary thread 1: Save Face ------------------------------
    def save_face_t(self,name):
        
        print(self.consoleFormatter.format("SAVE_FACE THREAD", "HEADER"))
        attempts = 0
        chances = 3
        if name == "charlie":
            self.host_saved = False
            self.waiting_host = True
            chances = 2
        elif name == "guest1":
            self.guest_1_saved = False
        while not self.saved_face and attempts<chances:
            self.saved_face  = self.tm.save_face(name, 3)
            if self.saved_face:
                if name == "charlie":
                    self.host_saved = True
                    self.waiting_host = False
                elif name == "guest1":
                    self.guest_1_saved = True
            attempts+=1
        if attempts==3:
            if name == "charlie":
                self.waiting_host = False
            

    # --------------------------- Complementary thread 2: Get hair color with gptvision ------------------------------
    def gpt_hair_t(self):
        
        print(self.consoleFormatter.format("GET HAIR THREAD", "HEADER"))
        
        gpt_vision_prompt = "Please answer about the person centered in the image: What is the hair color of the person? Answer only with the color for example: Black"
        answer = self.tm.img_description(gpt_vision_prompt)["message"]
        self.first_guest["hair"] = answer
        

    # --------------------------- Complementary thread 3: Get clothes color with gptvision ------------------------------
    def gpt_clothes_t(self):
        
        print(self.consoleFormatter.format("GET CLOTHES THREAD", "HEADER"))
        
        gpt_vision_prompt = "Please answer about the person centered in the image: What is the main color of the clothes this person is wearing? Answer only with the color for example: Black"
        answer = self.tm.img_description(gpt_vision_prompt)["message"]
        self.first_guest["clothes"] = answer

    # --------------------------- Complementary thread 4: Get the persons age and gender with perception ------------------------------
    def age_gender_t(self):
        
        print(self.consoleFormatter.format("GET ATTRIBUTES THREAD", "HEADER"))
        
        person_attributes = self.tm.get_person_description()
        self.first_guest["age"] = self.categorize_age(person_attributes["age"])
        self.first_guest["gender"] = person_attributes["gender"]
        self.first_guest["pronoun"] = "he" if person_attributes["gender"] == "Man" else "she"
    
    
    # --------------------------- Complementary function 1: Calculates ranges of age ------------------------------
    def categorize_age(self, age):
        if age < 18:
            category = "a teenager"
        elif age < 25:
            category = "a young adult"
        elif age < 35:
            category = "an adult"
        elif age < 50:
            category = "a middle aged adult"
        elif age < 65:
            category = "a senior"
        else:
            category = "an elder"
        return category

    # --------------------------- Complementary function 2: Point to desired angle ------------------------------

    def point_angle(self, angle):
        self.tm.go_to_pose("open_both_hands")
        if angle < 0:
            self.tm.go_to_pose("point_there_left",0.1)
            joint = "LElbowRoll"
        elif angle > 0:
            self.tm.go_to_pose("point_there_right",0.1)
            joint = "RElbowRoll"
        rospy.sleep(2)
        self.tm.set_angles_srv([joint], [math.radians(angle-10)], 0.1)
            
        
    # --------------------------- ROSPY CHECK THREAD ------------------------------
    def check_rospy(self):
        
        # Ends all processes if rospy is not running
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            
        print(self.consoleFormatter.format("Shutting down", "FAIL"))
        
        os._exit(os.EX_OK)

    # --------------------------- RUN FUNCTION TO START THE TASK CLASS CONSTRUCTOR ------------------------------
    def run(self):
        self.start()

# --------------------------- MAIN FUNCTION OF THE STICKLER TASK CLASS ------------------------------
if __name__ == "__main__":
    sm = RECEPTIONIST()
    sm.run()
    rospy.spin()
