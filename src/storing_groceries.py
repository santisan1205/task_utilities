#!/usr/bin/env python3
import os
import rospy
import threading
import ConsoleFormatter

from transitions import Machine
from task_module import Task_module as tm

from robot_toolkit_msgs.srv import set_angle_srv

class STORING_GROCERIES(object):
    def __init__(self) -> None:
        
        self.consoleFormatter = ConsoleFormatter.ConsoleFormatter()
        self.tm = tm(navigation=True, manipulation=True, speech=True, perception = True, pytoolkit=True)
        self.tm.initialize_node('STORING_GROCERIES')
 
        self.STATES = ['INIT', 'GO_2_TABLE', 'ASK_E_GRAB_OBJECT', 'GO_2_CABINET','ANALIZE_OBJECTS','CATEGORIZE_CABINETS' ,'PLACE_OBJECT', 'BACK_2_TABLE','END'] 

        self.TRANSITIONS = [
            {'trigger': 'zero', 'source': 'STORING_GROCERIES', 'dest': 'INIT'},
            {'trigger': 'go_2_table', 'source': 'INIT', 'dest': 'GO_2_TABLE'},
            {'trigger': 'analice_objects', 'source': 'GO_2_TABLE', 'dest': 'ANALIZE_OBJECTS'},
            {'trigger': 'ask_e_grab', 'source': 'ANALIZE_OBJECTS', 'dest': 'ASK_E_GRAB_OBJECT'},
            {'trigger': 'go_2_cabinet', 'source': 'ASK_E_GRAB_OBJECT', 'dest': 'GO_2_CABINET'},
            {'trigger': 'categorize', 'source': 'GO_2_CABINET', 'dest': 'CATEGORIZE_CABINETS'},
            {'trigger': 'place_first', 'source': 'CATEGORIZE_CABINETS', 'dest': 'PLACE_OBJECT'},
            {'trigger': 'place_object', 'source': 'GO_2_CABINET', 'dest': 'PLACE_OBJECT'},
            {'trigger': 'back_2_table', 'source': 'PLACE_OBJECT', 'dest': 'BACK_2_TABLE'},
            {'trigger': 'new_object', 'source': 'BACK_2_TABLE', 'dest': 'ASK_E_GRAB_OBJECT'},
            {'trigger': 'finish', 'source': 'PLACE_OBJECT', 'dest': 'END'},
            ]
        
        self.machine = Machine(model=self, states=self.STATES, transitions=self.TRANSITIONS, initial='STORING_GROCERIES')
        
        rospy_check = threading.Thread(target=self.check_rospy)
        rospy_check.start()
        
        # -------------------------------------------------------------------------------------------------------------------------------------------
        #                                                           ROS
        # -------------------------------------------------------------------------------------------------------------------------------------------

        rospy.wait_for_service("/pytoolkit/ALMotion/set_angle_srv")
        self.set_angle_srv = rospy.ServiceProxy("/pytoolkit/ALMotion/set_angle_srv",set_angle_srv)
        
        # -------------------------------------------------------------------------------------------------------------------------------------------
        #                                                           PARÁMETROS AJUSTABLES
        # -------------------------------------------------------------------------------------------------------------------------------------------

        # Movement Parameters
        self.fast_movement = 0.15
        self.normal_movement = 0.1
        self.slow_movement = 0.05
        
        # Relative angle to cabinet from the main table
        self.cabinet_angle = -90
        
        # Relative distance to cabinet from the main table
        self.cabinet_approach_distance = 0.0    
        
        # Objects list (just in case gpt vision fails)
        self.objects_list = ["apple", "tenis ball", "ketchup", "tuna can", "orange juice"]
        
        # Grab actions list dictionary by object
        self.grab_actions = {
            "small_object":["prepare_2_grab_small", "bowl_hands"],
            "one_hand":["grab_one_hand", "open_both_hands", "close_both_hands"]
        }
        
        
        self.first_time = True
        
        # -------------------------------------------------------------------------------------------------------------------------------------------
        #                                                            ESTADOS / TRANSICIONES
        # -------------------------------------------------------------------------------------------------------------------------------------------

    def on_enter_INIT(self): 
        print(self.consoleFormatter.format("INIT", "HEADER"))
        self.tm.go_to_pose("standard", self.fast_movement)
        self.tm.go_to_pose("default_head", self.fast_movement)
        # self.tm.set_current_place("house_door") #TODO CAMBIAR LUGAR DE INICIO EN LA ROBOCUP
        # self.tm.talk("Hello! I will assist you in storing groceries.", "English", wait=False)
        self.tm.initialize_pepper()
        self.tm.turn_camera("depth_camera","custom",1,15)
        self.go_2_table()
        
        
    
    def on_enter_GO_2_TABLE(self):
        print(self.consoleFormatter.format("GO_2_TABLE", "HEADER"))
        # self.tm.go_to_place("dining") #TODO CAMBIAR LUGAR DE LLEGADA EN LA ROBOCUP
        self.analice_objects()
        
        
    
    def on_enter_ANALIZE_OBJECTS(self):
        print(self.consoleFormatter.format("ANALIZE_OBJECTS", "HEADER"))
        self.tm.set_security_distance(False)
        self.tm.go_to_pose("down_head_storing")
        self.tm.talk("I arrived at the table, I will need your valuable help please!", language="English", wait=False)
        objets_prompt = "I am going to show you a series of objects on a table. Only consider the nearest table. Provide me with a list of the objects you observe, using simple names separated by commas. Additionally, the names should use underscores instead of spaces. An example output would be: 'orange, water_bottle, apple, milk.'"
        objects = self.tm.img_description(objets_prompt, "depth_camera", 2.0)
        self.objects_list = (objects["message"]).split(", ")
        self.ask_e_grab()



    def on_enter_ASK_E_GRAB_OBJECT(self):
        print(self.consoleFormatter.format("ASK_E_GRAB_OBJECT", "HEADER"))
        print(f"Items: {self.objects_list}")
        self.actual_item = self.objects_list[0]
        print(f"Actual item: {self.actual_item}")

        object_position_prompt = f"Briefly describe the location, appearance, and a general category of the {self.actual_item} relative to the other objects on the table, in one line."
        object_description = self.tm.img_description(object_position_prompt, "depth_camera", 2.0)
        say_description = object_description["message"]
        self.tm.talk(f"Please take the {self.actual_item}. {say_description}", "English", wait=False)
        
        object_type_prompt = (
        f"""I will provide you with an item and I need you to categorize this item strictly into one of the following categories: 'small_object' or 'one_hand'. Please use only one of these two categories in your response, and respond with the category name exactly as it is listed.

        - 'small_object': Small but relatively heavy or long items, such as a can, a beverage, or a banana.
        - 'one_hand': Small and lightweight items that can be comfortably held with one hand, such as a fruit like an apple or a small ball.

        Here is the item: {self.actual_item}""")
        self.actual_item_type = self.tm.answer_question(object_type_prompt)
        print(f"Actual item type: {self.actual_item_type}")
        
        actions = self.grab_actions[self.actual_item_type]
        print(f"Grab actions for {self.actual_item_type}: {actions}")
        for action in actions:
            self.tm.go_to_pose(action, self.normal_movement)
            if (action == "grab_one_hand"):
                self.tm.talk(f"Please give me the {self.actual_item} in my hand just like my tablet shows", "English", wait=False)
                rospy.sleep(7)
                
            elif (action == "prepare_2_grab_small"):
                self.tm.talk(f"Please give me the {self.actual_item} between my hands just like my tablet shows", "English", wait=False)
                rospy.sleep(7)
            rospy.sleep(2)
        self.tm.talk(f"Thank you!", "English", wait=False)
        
        self.go_2_cabinet()



    def on_enter_GO_2_CABINET(self):
        print(self.consoleFormatter.format("GO_2_CABINET", "HEADER"))
        self.tm.go_to_relative_point(0.0, 0.0, self.cabinet_angle)
        self.tm.go_to_pose("up_head_storing")
        rospy.sleep(2)
        self.tm.go_to_relative_point(self.cabinet_approach_distance, 0.0, 0.0)
        self.tm.talk(f"Now I can see the cabinet", "English", wait=False)
        
        if self.first_time:
            self.first_time = False
            self.categorize()
        else:
            self.place_object()



    def on_enter_CATEGORIZE_CABINETS(self):
        print(self.consoleFormatter.format("CATEGORIZE_CABINETS", "HEADER"))
        
        category_shelfs_prompt = ("I will show you a drawer containing various items. Each level has different types of items, "
                                "and I need you to assign a simple category name to each level based on the items inside. "
                                "For example, if one level contains drinks, label it 'drinks'; if another contains canned goods, "
                                "label it 'canned_goods,' and so on. Please return only the category names, separated by commas and "
                                "using underscores instead of spaces. An example output should be: 'drinks, canned_goods, junk_food'.")
        
        try:
            category_recognition_request = self.tm.img_description(category_shelfs_prompt, "front_camera", 2.0)
            if "message" not in category_recognition_request:
                raise KeyError("The response from img_description does not contain 'message'")
            
            self.shelfs_categories = category_recognition_request["message"].split(", ")
            
            cabinet_positions = [0.6, 0.0, -0.6]
            self.cabinets = {}
            for i, category in enumerate(self.shelfs_categories):
                if i < len(cabinet_positions):
                    self.cabinets[category] = cabinet_positions[i]
                else:
                    self.cabinets[category] = -0.6 - 0.6 * (i - 2)
            
            self.place_first()
        
        except KeyError as e:
            print(f"KeyError: {e}")
            # Maneja el error apropiadamente
        except Exception as e:
            print(f"An unexpected error occurred: {e}")
            # Maneja el error apropiadamente

        
        
    def on_enter_PLACE_OBJECT(self):
        print(self.consoleFormatter.format("PLACE_OBJECT", "HEADER"))
        
        print(f"Available categories: {self.shelfs_categories}")
        print(f"Actual item: {self.actual_item}")
        
        place_to_store_promt = f"Classify the object: {self.actual_item}. Choose from the following categories: {', '.join(self.shelfs_categories)}. Respond only with the corresponding category. An example output is: {self.shelfs_categories[0]}"
        place_shelf = self.tm.answer_question(place_to_store_promt)
        
        print(f"Selected category for {self.actual_item}: {place_shelf}")
        
        self.tm.talk(f"Please, take the {self.actual_item} to place it in the {place_shelf} shelf.", "English", False)
        rospy.sleep(2)
        self.tm.go_to_pose("open_both_hands", self.normal_movement)
        rospy.sleep(2)
        self.tm.go_to_pose("standard", self.normal_movement)
        self.tm.talk(f"I will point to the {place_shelf} shelf.", "English", False)
        
        print(f"Cabinets dictionary: {self.cabinets}")
        
        if place_shelf in self.cabinets:
            print(f"Setting angle for {place_shelf} shelf")
            self.set_angle_srv(["RShoulderPitch"], [self.cabinets[place_shelf]], self.normal_movement)
        else:
            print(f"Error: {place_shelf} no es una clave válida en cabinets")
            self.set_angle_srv(["RShoulderPitch"], [0.0], self.normal_movement)  # Posición por defecto
        
        rospy.sleep(5)
        self.objects_list.pop(0)
        if self.objects_list == []:
            self.finish()
        else:
            self.back_2_table()
            
            
        
    def on_enter_BACK_2_TABLE(self):
        print(self.consoleFormatter.format("BACK_2_TABLE", "HEADER"))
        self.tm.go_to_relative_point(-(self.cabinet_approach_distance), 0.0, 0.0)
        rospy.sleep(2)
        self.tm.go_to_relative_point(0.0, 0.0, -(self.cabinet_angle))
        self.new_object()
        


    def on_enter_END(self):
        print(self.consoleFormatter.format("END", "HEADER"))
        self.tm.talk("I have finished storing the groceries! Thanks for your help!", "English", wait=False)
        os._exit(os.EX_OK)



    def check_rospy(self):
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
        print(self.consoleFormatter.format("Shutting down", "FAIL"))
        os._exit(os.EX_OK)
    
    
    
    def run(self):
        while not rospy.is_shutdown():
            self.zero()

        
    # -------------------------------------------------------------------------------------------------------------------------------------------
    #                                                         FUNCIÓN PRINCIPAL
    # -------------------------------------------------------------------------------------------------------------------------------------------
    
if __name__ == "__main__":
    sm = STORING_GROCERIES()
    sm.run()
    rospy.spin()
