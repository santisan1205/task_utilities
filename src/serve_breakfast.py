#!/usr/bin/env python3
import os
import rospy
import threading
from transitions import Machine
from task_module import Task_module as tm
from robot_toolkit_msgs.srv import set_angle_srv, set_security_distance_srv
import ConsoleFormatter

class ServeBreakfast(object):
    def __init__(self) -> None:
        self.console_formatter = ConsoleFormatter.ConsoleFormatter()
        self.tm = tm(navigation=True, manipulation=True, speech=True, perception=True, pytoolkit=True)
        self.tm.initialize_node('SERVE_BREAKFAST')
        self.states = ['INIT', 'GO_TO_CUPBOARD', 'LOOK_4_ITEM', 'GRAB_OBJECT', 'GO_TO_DROP_PLACE', 'DROP', 'SERVE', 'END']
        
        self.transitions = [
            {'trigger': 'zero', 'source': 'SERVE_BREAKFAST', 'dest': 'INIT'},
            {'trigger': 'start', 'source': 'INIT', 'dest': 'GO_TO_CUPBOARD'},
            {'trigger': 'scan_for_objects', 'source': 'GO_TO_CUPBOARD', 'dest': 'LOOK_4_ITEM'},
            {'trigger': 'grab_ingredient', 'source': 'LOOK_4_ITEM', 'dest': 'GRAB_OBJECT'},
            {'trigger': 'go_to_drop_place', 'source': 'GRAB_OBJECT', 'dest': 'GO_TO_DROP_PLACE'},
            {'trigger': 'serve', 'source': 'GO_TO_DROP_PLACE', 'dest': 'SERVE'},
            {'trigger': 'drop', 'source': 'SERVE', 'dest': 'DROP'},
            {'trigger': 'again', 'source': 'DROP', 'dest': 'GO_TO_CUPBOARD'},
            {'trigger': 'end', 'source': 'SERVE', 'dest': 'END'}
        ]
        
        self.machine = Machine(model=self, states=self.states, transitions=self.transitions, initial='SERVE_BREAKFAST')
        
        rospy_check = threading.Thread(target=self.check_rospy)
        rospy_check.start()
        
        self.fast_movement = 0.5
        self.normal_movement = 0.1
        self.slow_movement = 0.05
        
        self.items = ["bowl", "cereal_box", "milk_carton"] 
        
        # Angulos para ver los items en kitchen
        self.relative_item_angle = {
            "bowl": 0,
            "cereal_box": 0,
            "milk_carton": 0
        }
                
        self.grab_items_poses = {
            "bowl": ["open_both_hands", "prepare_2_bowl", "grab_bowl", "bowl_hands", "grab_bowl_up" ], # DONE
            "cereal_box": ["open_both_hands", "prepare_2_grab_mid", "grab_mid","close_both_hands" , "grab_mid_up", "carry_cereal"], # DONE
            "milk_carton": ["open_both_hands", "prepare_2_grab_big", "grab_big", "close_both_hands" ,"grab_big_up" "carry_milk"] # DONE
        }
        
        # Izquierda positivo - Derecha negativo
        self.drop_and_serve_position = {
            "bowl": 0.0,
            "milk_carton": 0.25,
            "cereal_box": -0.25
        }
        
        
        self.serve_items_poses = {
            "milk_carton": ["grab_big_up", "grab_big", "serve_milk", "grab_big", "grab_big_up"], # DONE
            "cereal_box": ["grab_mid", "grab_mid_up","serve_cereal", "grab_mid_up", "grab_mid"] # DONE
        }
        
        self.drop_items_poses = {
            "bowl": ["grab_bowl","open_both_hands", "prepare_2_grab_big"], #DONE
            "milk_carton": ["grab_big", "open_both_hands", "drop"], # DONE
            "cereal_box": ["grab_mid", "open_both_hands", "drop"] # DONE
        }
                
        self.consoleFormatter=ConsoleFormatter.ConsoleFormatter()
        
        # Positivo adelante - Negativo atras
        self.relative_drop_position=0.27
        
        self.crouch_4_drop = -0.3 # Cambiar segun la altura de la mesa de dining
        
        self.is_first = True
        
        # ---------------------------------------------------------------------------
        #                       SERVICES
        # ---------------------------------------------------------------------------
        
        print(self.consoleFormatter.format("Waiting for pytoolkit/ALMotion/set_orthogonal_security_distance_srv...", "WARNING"))
        rospy.wait_for_service("/pytoolkit/ALMotion/set_orthogonal_security_distance_srv")
        self.set_orthogonal_security_srv = rospy.ServiceProxy("/pytoolkit/ALMotion/set_orthogonal_security_distance_srv",set_security_distance_srv)

        print(self.consoleFormatter.format("Waiting for pytoolkit/ALMotion/set_tangential_security_distance_srv...", "WARNING"))
        rospy.wait_for_service("/pytoolkit/ALMotion/set_tangential_security_distance_srv")
        self.set_tangential_security_srv = rospy.ServiceProxy("/pytoolkit/ALMotion/set_tangential_security_distance_srv",set_security_distance_srv)
        
        rospy.wait_for_service("/pytoolkit/ALMotion/set_angle_srv")
        self.set_angle_srv = rospy.ServiceProxy("/pytoolkit/ALMotion/set_angle_srv",set_angle_srv)
        
        
        
    # ---------------------------------------------------------------------------
    #                       ESTADOS / TRANSICIONES
    # ---------------------------------------------------------------------------

    def on_enter_INIT(self):
        self.tm.go_to_pose("standard", self.normal_movement)
        print(self.consoleFormatter.format("INIT", "HEADER"))
        self.tm.set_current_place("house_door")
        self.tm.initialize_pepper()
        self.tm.talk("Hi! Today I would serve you a cereal", "English", wait=False)
        self.start()



    def on_enter_GO_TO_CUPBOARD(self):
        print(self.consoleFormatter.format("GO_TO_CUPBOARD", "HEADER"))
        self.tm.enable_security_proxy()
        self.tm.go_to_pose("standard", self.normal_movement)
        self.tm.talk("Now I will go to the Kitchen", "English", wait=False)
        self.set_orthogonal_security_srv(0.3)
        self.set_tangential_security_srv(0.05)
        self.tm.go_to_place("kitchen", lower_arms=False)
        self.tm.go_to_place("down_head", lower_arms=False)
        if self.is_first:
            response = self.tm.img_description(
                    f"Describe the positions, sizes, and colors of the spoon, bowl, cereal box, and milk that you see in front of you on the table. Make your description detailed but concise.",
                    "front_camera")
            res = response["message"]
            self.tm.talk(f"{res}", "English", wait=True)
            self.is_first = False
        self.scan_for_objects()

    
    
    def on_enter_LOOK_4_ITEM(self):
        print(self.consoleFormatter.format("LOOK_4_ITEM", "HEADER"))
        self.actual_item=self.items[0]
        self.tm.go_to_pose("down_head")
        self.tm.talk(f"I will look for the {self.actual_item}.", "English", wait=False)
        actual_relative_angle = self.relative_item_angle[self.actual_item]
        self.tm.go_to_relative_point(0.0,0.0,actual_relative_angle)
        response = self.tm.img_description(
                f"Describe where is the {self.actual_item} you see in front of you, add some details about the size and color. "
                "Then describe the place where it is located naming objects around it. "
                "Don't add any other words to your response. Give a short answer of maximum 2 lines. "
                "If you don't see a {object_name}, answer just with an empty string and don't say sorry or anything more.",
                "front_camera")
        res = response["message"]
        self.tm.go_to_pose("default_head")
        if response["message"] != "":
            self.tm.talk(f"{res}. Just right there", "English", wait=False)
        else:
            self.tm.talk(f"I see the {self.actual_item} right there", "English", wait=False)
        self.tm.go_to_pose("point_there", self.normal_movement)
        self.items.pop(0)
        self.grab_ingredient()



    def on_enter_GRAB_OBJECT(self):
        print(self.consoleFormatter.format("GRAB_OBJECT", "HEADER"))
        self.tm.set_security_distance(False)
        actions = self.grab_items_poses[self.actual_item]
        self.tm.show_image(f"http://raw.githubusercontent.com/SinfonIAUniandes/Image_repository/main/grab_{self.actual_item}.jpeg")
        self.tm.go_to_pose("open_both_hands")
        
        if self.actual_item == "bowl":
            self.tm.talk("Could you please put the spoon in the bowl just like appear in my tablet?", "English", wait=False)     
            rospy.sleep(5)
            self.tm.talk("Thank you!", "English", wait=False) 
        
        for action in actions:
            print(action)
            if action == "prepare_2_bowl" :
                self.tm.go_to_pose(action, self.slow_movement)
                self.tm.talk("Please place the bowl in my hands just like the image in my tablet shows! Please hold it until I tell you: Thank you and close my hands.", wait=False)
                rospy.sleep(7)
                self.tm.talk("Make sure that the spoon is above one of my fingers please", wait=False)
                rospy.sleep(3)
                self.tm.talk("Thank you!", wait=False)

            if action == "prepare_2_grab_mid" or action == "prepare_2_grab_big":
                self.tm.go_to_pose(action, self.slow_movement)
                if self.actual_item == "cereal_box":
                    self.tm.talk("To help me serve the cereal, make sure to give me the box already opened, with the open side in my right hand.", "English", wait=False)
                elif self.actual_item == "milk_carton":
                    self.tm.talk("To help me serve the milk, make sure to give me the carton already opened, with the open side in my left hand.", "English", wait=False)
                rospy.sleep(3)
                self.tm.talk(f"Now that you have ensured the {self.actual_item} is open, please hold it in the middle of my hands as shown in the image on my tablet until I say 'Thank you' and my hands are closed.", "English", wait=False)
                rospy.sleep(7)
            
            if action  == "carry_cereal" or action  == "carry_milk" or action  == "grab_bowl_up":
                self.carry_to_serve = True
                self.tm.talk("Thank you! again", "English", wait=False) 
                rospy.sleep(2)
                carry_thread = threading.Thread(target=self.carry_thread,args=[action])
                carry_thread.start()
    
            self.tm.go_to_pose(action, self.slow_movement)  
            rospy.sleep(2)
        
        self.tm.show_words_proxy()
        self.go_to_drop_place()
        


    def on_enter_GO_TO_DROP_PLACE(self):
        print(self.consoleFormatter.format("GO_TO_DROP_PLACE", "HEADER"))
        self.tm.enable_security_proxy()
        self.set_orthogonal_security_srv(0.3)
        self.set_tangential_security_srv(0.05)
        self.tm.talk("On my way to the dining room", "English", wait=False)
        self.tm.set_move_arms_enabled(False)
        self.tm.go_to_place("dining", lower_arms=False)
        self.carry_to_serve = False
        self.serve()
        
    def on_enter_SERVE(self):
        print(self.consoleFormatter.format("SERVE", "HEADER"))
        drop_relative_point = self.drop_and_serve_position[self.actual_item]
        self.tm.go_to_relative_point(0.0, drop_relative_point, 0.0)
        self.tm.set_security_distance(False)
        self.tm.go_to_relative_point(self.relative_drop_position, 0.0, 0.0)
        if self.actual_item == "cereal_box" or self.actual_item == "milk_carton":
            self.tm.talk(f"I will serve the {self.actual_item}", "English", wait=False) 
            actions = self.serve_items_poses[self.actual_item]
            for action in actions:
                print(action)
                self.tm.go_to_pose(action, self.slow_movement)
                if action == "serve_cereal":
                    rospy.sleep(2)
                    self.shake_cereal()
                rospy.sleep(1)
        if self.items == []:
            self.tm.go_to_relative_point(-(self.relative_drop_position), 0.0, 0.0)
            self.end()
        else:
            self.drop()


    def on_enter_DROP(self):
        print(self.consoleFormatter.format("DROP", "HEADER"))
    
        actions = self.drop_items_poses[self.actual_item]  
        self.set_angle_srv(["HipPitch"], [self.crouch_4_drop], self.normal_movement)
        
        for action in actions:
            self.tm.go_to_pose(action, self.slow_movement)
            rospy.sleep(2)
        
        self.tm.set_security_distance(True)
        self.set_orthogonal_security_srv(0.3)
        self.set_tangential_security_srv(0.05)
        self.tm.go_to_relative_point(-(self.relative_drop_position), 0.0, 0.0)
        self.again()

            
    

    def on_enter_END(self):
        print(self.consoleFormatter.format("END", "HEADER"))
        self.tm.go_to_pose("standard", self.normal_movement)
        self.tm.talk("I finished serving breakfast, enjoy it", "English", wait=False)
        return

    def check_rospy(self):
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
        print(self.console_formatter.format("Shutting down", "FAIL"))
        os._exit(os.EX_OK)

    def carry_thread(self, pose):
        print(self.consoleFormatter.format("Carring", "OKBLUE"))
        while self.carry_to_serve:
            self.tm.go_to_pose(pose)
            print(pose)
            rospy.sleep(1)
        
    def shake_cereal(self):
        print(self.consoleFormatter.format("Shaking", "OKBLUE"))
        shake_poses = ["shake_cereal_1", "shake_cereal_2"]
        for _ in range(10):
            for pose in shake_poses:
                self.tm.go_to_pose(pose, self.fast_movement)

    def run(self):
        while not rospy.is_shutdown():
            self.zero()
    # ---------------------------------------------------------------------------
    #                       FUNCIÓN PRINCIPAL
    # ---------------------------------------------------------------------------
    
if __name__ == "__main__":
    serve_breakfast = ServeBreakfast()
    serve_breakfast.run()
    rospy.spin()
