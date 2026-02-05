#!/usr/bin/env python3
import os
import ast
import rospy
import threading
import ConsoleFormatter

from transitions import Machine
from task_module import Task_module as tm

from robot_toolkit_msgs.srv import set_angle_srv

class CLEAN_THE_TABLE(object):
    def __init__(self) -> None:
        
        self.consoleFormatter = ConsoleFormatter.ConsoleFormatter()
        self.tm = tm(navigation=True, manipulation=True, speech=True, perception = True, pytoolkit=True)
        self.tm.initialize_node('CLEAN_THE_TABLE')
 
        self.STATES = ['INIT', 'LOOK_2_TABLE', 'GRAB_OBJECT', 'GO_2_DISHWASHER', 'BACK_2_TABLE', 'END'] 

        self.TRANSITIONS = [{'trigger': 'zero', 'source': 'CLEAN_THE_TABLE', 'dest': 'INIT'},
                            {'trigger': 'start', 'source': 'INIT', 'dest': 'LOOK_2_TABLE'},
                            {'trigger': 'grab_object', 'source': 'LOOK_2_TABLE', 'dest': 'GRAB_OBJECT'},
                            {'trigger': 'turn_around', 'source': 'GRAB_OBJECT', 'dest': 'GO_2_DISHWASHER'},
                            {'trigger': 'finish', 'source': 'GO_2_DISHWASHER', 'dest': 'END'},
                            {'trigger': 'back_2_table', 'source': 'GO_2_DISHWASHER', 'dest': 'BACK_2_TABLE'},
                            {'trigger': 'grab_again', 'source': 'BACK_2_TABLE', 'dest': 'GRAB_OBJECT'},
                            {'trigger': 'force_end', 'source': 'LOOK_2_TABLE', 'dest': 'END'}]
        
        self.machine = Machine(model=self, states=self.STATES, transitions=self.TRANSITIONS, initial='CLEAN_THE_TABLE')
        
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
        
        # Poses to grab items by item
        self.grab_items_poses = {
        "dish": ["open_both_hands","prepare_2_grab", "grab_plate_vertically"],
        "bowl": ["open_both_hands","prepare_2_grab", "grab_plate_vertically","bowl_hands"],
        "cup": ["open_both_hands","prepare_2_grab", "grab_plate_vertically","almost_close_both_hands"],
        "spoon": ["carry_cutlery", "close_both_hands"],
        "fork":["carry_cutlery", "close_both_hands"]
        }
        
        # Place items on dishwasher distances
        self.place_items_distances = {
        "dish": 0.0,
        "bowl": 0.1,
        "cup": 0.2,
        "spoon": -0.2,
        "fork":-0.2
        }
        
        # Grab table position
        self.relative_table_rotation = 0
        self.relative_table_approach = 0.0
        
        # In Dishwasher
        self.turn_around_2_dishwasher = -90
        self.relative_dishwasher_approach = 0.6
        self.crouch_4_dishwasher = -2.0 # 10 grados hacia adelante
        
        self.place_items_poses = {
        "dish": ["grab_plate_vertically", "prepare_2_grab"],
        "bowl": ["grab_plate_vertically","open_both_hands", "prepare_2_grab"],
        "cup": ["grab_plate_vertically", "open_both_hands", "prepare_2_grab"],
        "spoon": ["open_both_hands"],
        "fork":["open_both_hands"]
        }
        
        # Relatives positions from dishwasher to table
        self.dishwasher_2_table_rotation = -90
        self.dishwasher_table_approach = 0.6
        self.dishwasher_2_table_rotation_back = 180
        
        # -------------------------------------------------------------------------------------------------------------------------------------------
        #                                                            ESTADOS / TRANSICIONES
        # -------------------------------------------------------------------------------------------------------------------------------------------

    def on_enter_INIT(self): 
        print(self.consoleFormatter.format("INIT", "HEADER"))
        self.tm.go_to_pose("standard", self.fast_movement)
        self.tm.go_to_pose("default_head", self.fast_movement)
        # self.tm.set_current_place("kitchen")
        self.tm.talk("Hi, I am going to clean the table.", "English", wait=False)
        self.tm.initialize_pepper()
        self.tm.set_security_distance(False)
        self.start()

    def on_enter_LOOK_2_TABLE(self):
        print(self.consoleFormatter.format("LOOK_2_TABLE", "HEADER"))
        self.tm.go_to_relative_point(0.0,0.0,self.relative_table_rotation)
        rospy.sleep(1)
        self.tm.go_to_pose("down_head", self.fast_movement)
        self.tm.go_to_relative_point(self.relative_table_approach,0.0,0.0)
        response = self.tm.img_description("You are observing a table set for a meal through a camera. On the table, there are various items which may include a cup, bowl, dish, spoon, and fork. Your task is to list exactly and only the objects you see from these options. Please respond by typing out the response directly as a Python list, including the square brackets and using quotes for each item. The format should look like this: ['object1', 'object2', ...]. For example, if you see a spoon, fork, and cup on the table, your response should be exactly: ['spoon', 'fork', 'cup']. Ensure your response is structured strictly as a list for direct use in a Python program.", "front_camera")
        try:
            self.items = ast.literal_eval(response["message"])
            if not isinstance(self.items, list):
                raise ValueError("Invalid response")
        except (SyntaxError, ValueError) as e:
            print(f"Error: {e}")
            self.items = ["bowl","spoon", "dish"]
        self.items = ["bowl","spoon", "dish", "fork", "cup"] # TODO Esto está quemado
        if self.items != []:
            self.tm.talk("I saw over the table the next items", "English", wait=False)
            for item in self.items:
                self.tm.talk(f"a {item}", "English", wait=False)
            self.grab_object()
        else:
            self.tm.talk("I can't see any object to carry to the dishwasher", "English", wait=False)
            self.force_end()
        
    def on_enter_GRAB_OBJECT(self):
        print(self.consoleFormatter.format("GRAB_OBJECT", "HEADER"))
        self.tm.go_to_pose("default_head", self.fast_movement)
        self.actual_item = self.items[0]
        self.tm.show_image(f"http://raw.githubusercontent.com/SinfonIAUniandes/Image_repository/main/grab_{self.actual_item}.jpeg")
        grab_poses = self.grab_items_poses[self.actual_item]
        for pose in grab_poses:
            self.tm.go_to_pose(pose, self.normal_movement)
            if pose == "prepare_2_grab" or pose == "close_both_hands":
                self.tm.talk(f"Please give me and still holding the {self.actual_item} just like you can see in my tablet until I can grab propertly", "English", wait=False)
                rospy.sleep(6)
            rospy.sleep(2)
        
        self.turn_around()
                
    def on_enter_GO_2_DISHWASHER(self):
        print(self.consoleFormatter.format("GO_2_DISHWASHER", "HEADER"))
        self.tm.go_to_relative_point(0.0,0.0,self.turn_around_2_dishwasher)
        self.tm.go_to_relative_point(self.relative_dishwasher_approach,0.0,0.0)
        self.set_angle_srv(["HipPitch"], [self.crouch_4_dishwasher], self.normal_movement)
        place_distance = self.place_items_distances[self.actual_item]
        self.tm.go_to_relative_point(0.0, place_distance, 0.0)
        place_poses = self.place_items_poses[self.actual_item]
        for pose in place_poses:
            self.tm.go_to_pose(pose, self.slow_movement)
            rospy.sleep(2)
        rospy.sleep(4)
        self.tm.talk(f"I just dropped the {self.actual_item} on the dishwasher", "English", wait=False)
        self.items.pop(0)
        self.tm.go_to_pose("standard", self.normal_movement)
        if len(self.items) == 0:
            self.finish()
        else:
            self.tm.talk("I will go now for the next item", "English", wait=False)
            self.back_2_table()
            
    def on_enter_BACK_2_TABLE(self):
        print(self.consoleFormatter.format("BACK_2_TABLE", "HEADER"))
        self.tm.go_to_relative_point(0.0,0.0,self.dishwasher_2_table_rotation_back)
        self.tm.go_to_relative_point(self.relative_dishwasher_approach,0.0,0.0)
        self.tm.go_to_relative_point(0.0,0.0,self.dishwasher_2_table_rotation)
        self.grab_again()

    def on_enter_END(self):
        print(self.consoleFormatter.format("END", "HEADER"))
        self.tm.talk("I finished cleaning the table")
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
    sm = CLEAN_THE_TABLE()
    sm.run()
    rospy.spin()
