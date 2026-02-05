#!/usr/bin/env python3

# --------------- GENERAL LIBRARIES IMPORTS ---------------
import os
import time
import rospy
import threading
import ConsoleFormatter
from transitions import Machine
from task_module import Task_module as tm

from std_srvs.srv import SetBool
from robot_toolkit_msgs.srv import tablet_service_srv, set_stiffnesses_srv, set_stiffnesses_srvRequest, set_open_close_hand_srv, set_open_close_hand_srvRequest,set_angle_srv, set_angle_srvRequest
from robot_toolkit_msgs.msg import leds_parameters_msg, touch_msg, set_angles_msg, speech_recognition_status_msg

# --------------- MESSAGES AND SERVICES IMPORTS ---------------
from std_msgs.msg import String

# --------------- CARRY MY LUGGAGE TASK CLASS DEFINITION ---------------
class CARRY_MY_LUGGAGE(object):
    
    # --------------- Constructor ---------------    
    def __init__(self) -> None:
        self.consoleFormatter = ConsoleFormatter.ConsoleFormatter()
        self.tm = tm(navigation=False, manipulation=False, perception=False, speech=True, pytoolkit=True)
        self.tm.initialize_node("pose_generator")

        # --------------- Machine States ---------------

        self.STATES = [
            "INIT",
            "LOOK_FOR_BAG",
            "GRAB_BAG",
            "FOLLOW_YOU",
            "GO_BACK",
            "END",
        ]
        
        # --------------- Machine Transitions ---------------
        
        self.TRANSITIONS = [
            {"trigger": "zero", "source": "CARRY_MY_LUGGAGE", "dest": "INIT"},
            {"trigger": "start", "source": "INIT", "dest": "LOOK_FOR_BAG"},
            {"trigger": "grab_bag", "source": "LOOK_FOR_BAG", "dest": "GRAB_BAG"},
            {"trigger": "grab_bag_again", "source": "GRAB_BAG", "dest": "GRAB_BAG"},
            {"trigger": "follow_you", "source": "GRAB_BAG", "dest": "FOLLOW_YOU"},
            {"trigger": "go_back", "source": "FOLLOW_YOU", "dest": "GO_BACK"},
            {"trigger": "end", "source": "GO_BACK", "dest": "END"},
        ]
        
        # --------------- Machine Declaration ---------------

        self.machine = Machine(
            model=self,
            states=self.STATES,
            transitions=self.TRANSITIONS,
            initial="CARRY_MY_LUGGAGE",
        )
        
        # --------------- ROSPY Check ---------------
        
        self.angles = []
        self.can_move_head = False
        rospy_check = threading.Thread(target=self.check_rospy)
        rospy_check.start()


    # --------------------------------------------- STATE FUNCTIONS DEFINITION ---------------------------------------------
    
        
    # === Parameters whisper ===
    def setLedsColor(self,r,g,b):
        """
        Function for setting the colors of the eyes of the robot.
        Args:
        r,g,b numbers
            r for red
            g for green
            b for blue
        """
        ledsPublisher = rospy.Publisher('/leds', leds_parameters_msg, queue_size=10)
        ledsMessage = leds_parameters_msg()
        ledsMessage.name = "FaceLeds"
        ledsMessage.red = r
        ledsMessage.green = g
        ledsMessage.blue = b
        ledsMessage.time = 0
        ledsPublisher.publish(ledsMessage)
        rospy.sleep(0.2)
     
    # --------------- FIRST STATE: INIT ---------------
    
    def on_enter_INIT(self):
        
        
        # --------------- INIT STATE PROCCEDURE ---------------
        self.tm.show_words_proxy()
        rospy.sleep(3)
        print(self.consoleFormatter.format("INIT", "HEADER"))
        request = set_open_close_hand_srvRequest()
        request.hand = "All"
        request.state = "False"
        rospy.wait_for_service("/pytoolkit/ALMotion/toggle_smart_stiffness_srv")
        self.toggle_smart_stiffness = rospy.ServiceProxy("/pytoolkit/ALMotion/toggle_smart_stiffness_srv", SetBool)
        self.toggle_smart_stiffness(False)
        rospy.wait_for_service("/pytoolkit/ALAutonomousBlinking/toggle_blinking_srv")
        self.toggle_blinking = rospy.ServiceProxy("/pytoolkit/ALAutonomousBlinking/toggle_blinking_srv", SetBool)
        self.toggle_blinking(True)
        self.toggle_blinking(False)
        self.toggle_blinking(False)
        rospy.wait_for_service("/pytoolkit/ALMotion/toggle_breathing_srv")
        toggle_breathing_proxy = rospy.ServiceProxy("/pytoolkit/ALMotion/toggle_breathing_srv", set_open_close_hand_srv)
        toggle_breathing_proxy(request)
        self.open_close_hand_proxy = rospy.ServiceProxy("/pytoolkit/ALMotion/set_open_close_hand_srv", set_open_close_hand_srv)
        self.tm.stop_tracker_proxy()
        self.motion_set_stiffnesses_client = rospy.ServiceProxy("pytoolkit/ALMotion/set_stiffnesses_srv", set_stiffnesses_srv)
        req_stiffnesses = set_stiffnesses_srvRequest()
        req_stiffnesses.names = "Body"
        req_stiffnesses.stiffnesses = 1
        self.motion_set_stiffnesses_client(req_stiffnesses)
        self.set_arms_security = rospy.ServiceProxy("pytoolkit/ALMotion/set_arms_security_srv", SetBool)
        self.set_arms_security(False)
        self.grabando = False
        self.subscriber_angles = rospy.Subscriber("/pytoolkit/ALMotion/get_angles",set_angles_msg,self.callback_get_angles)
        self.headSensorSubscriber = rospy.Subscriber("/touch", touch_msg, self.callback_head_sensor_subscriber)
        subscriber = rospy.Subscriber("/pytoolkit/ALSpeechRecognition/status",speech_recognition_status_msg,self.callback_hot_word)
        hot_word_language_srv = rospy.ServiceProxy("/pytoolkit/ALSpeechRecognition/set_hot_word_language_srv", tablet_service_srv)
        hot_word_language_srv("Spanish")
        self.tm.hot_word(["graba", "detente", "repite","abre","cierra"],thresholds=[0.4, 0.4,0.38,0.4,0.4])
        self.setLedsColor(165,165,0)
        self.setLedsColor(165,165,0)
        self.setLedsColor(165,165,0)
        self.tm.talk("Ya!",language="Spanish")
        #self.tm.talk("Generador de poses activado. Cuando mis ojos esten naranjas mis extremidades estan fijas, cuando estan verdes mis extremidades se pueden mover, para empezar a mover mi cabeza toca el sensor en la mitad de mi cabeza y cuando acabes vuelve a tocarlo.", language="Spanish", wait=True)
        #self.tm.talk("Para mover uno de mis brazos toca el sensor exterior y mueve el brazo con cuidado, una vez sueltes el sensor mi brazo se quedara fijo", language="Spanish", wait=True)    
            
        while True:
            rospy.sleep(1)
        
        # Moving to LOOK_FOR_BAG state
        self.start()


    def callback_head_sensor_subscriber(self, msg: touch_msg):
        if "head_middle" in msg.name:
            if msg.state:
                #Apagar que se pueda mover la cabeza
                if self.can_move_head:
                    self.tm.talk("Cabeza fija", language="Spanish", wait=False)
                    self.can_move_head = False
                    self.setLedsColor(165,165,0)
                    req_stiffnesses = set_stiffnesses_srvRequest()
                    req_stiffnesses.names = "Head"
                    req_stiffnesses.stiffnesses = 1
                    self.motion_set_stiffnesses_client(req_stiffnesses)
                #Encender que se pueda mover la cabeza
                else:
                    self.tm.talk("Cabeza suelta", language="Spanish", wait=False)
                    self.can_move_head = True
                    self.setLedsColor(0,255,0)
                    req_stiffnesses = set_stiffnesses_srvRequest()
                    req_stiffnesses.names = "Head"
                    req_stiffnesses.stiffnesses = 0
                    self.motion_set_stiffnesses_client(req_stiffnesses)
        elif "hand_left_back" in msg.name:
            if msg.state:
                self.tm.talk("Brazo izquierdo suelto", language="Spanish", wait=False)
                self.setLedsColor(0,255,0)
            else:
                self.tm.talk("Brazo izquierdo fijo", language="Spanish", wait=False)
                self.setLedsColor(165,165,0)
                print(self.angles)
            req_stiffnesses = set_stiffnesses_srvRequest()
            req_stiffnesses.names = "LArm"
            req_stiffnesses.stiffnesses = 0 if msg.state else 1
            self.motion_set_stiffnesses_client(req_stiffnesses)
        elif "hand_right_back" in msg.name:  
            if msg.state:
                self.tm.talk("Brazo derecho suelto", language="Spanish", wait=False)
                self.setLedsColor(0,255,0)
            else:
                self.tm.talk("Brazo izquierdo fijo", language="Spanish", wait=False)
                self.setLedsColor(165,165,0)
                print(self.angles)
            req_stiffnesses = set_stiffnesses_srvRequest()
            req_stiffnesses.names = "RArm"
            req_stiffnesses.stiffnesses = 0 if msg.state else 1
            self.motion_set_stiffnesses_client(req_stiffnesses)

    def callback_hot_word(self,data):
        word = data.status
        print(word, "listened")
        if word=="graba":
            self.grabando = True
            self.angles = []
            toggle_msg =  set_angle_srvRequest()
            toggle_msg.name = ["Body"]
            toggle_msg.angle = []
            toggle_msg.speed = 0
            self.tm.toggle_get_angles_topic_srv(toggle_msg)
        elif word=="para":
            self.grabando = False
            toggle_msg =  set_angle_srvRequest()
            toggle_msg.name = ["None"]
            toggle_msg.angle = []
            toggle_msg.speed = 0
            self.tm.toggle_get_angles_topic_srv(toggle_msg)
        elif word=="repite":
            self.grabando = False
            self.tm.set_angles_srv(joints=["Body"], angles= self.angles.pop(0), speed= 0.1)
            rospy.sleep(3)
            for angle_idx in range(0, len(self.angles), 3):
                angle = self.angles[angle_idx]
                self.tm.set_angles_srv(joints=["Body"], angles=angle, speed=0.1)
        elif word=="cierra":
            req_stiffnesses = set_stiffnesses_srvRequest()
            req_stiffnesses.names = "LHand"
            req_stiffnesses.stiffnesses = 1
            self.motion_set_stiffnesses_client(req_stiffnesses)
            req_stiffnesses = set_stiffnesses_srvRequest()
            req_stiffnesses.names = "RHand"
            req_stiffnesses.stiffnesses = 1
            self.motion_set_stiffnesses_client(req_stiffnesses)
            request = set_open_close_hand_srvRequest()
            request.hand = "both"
            request.state = "close"
            self.open_close_hand_proxy(request)
        elif word=="abre":
            req_stiffnesses = set_stiffnesses_srvRequest()
            req_stiffnesses.names = "LHand"
            req_stiffnesses.stiffnesses = 1
            self.motion_set_stiffnesses_client(req_stiffnesses)
            req_stiffnesses = set_stiffnesses_srvRequest()
            req_stiffnesses.names = "RHand"
            req_stiffnesses.stiffnesses = 1
            self.motion_set_stiffnesses_client(req_stiffnesses)
            request = set_open_close_hand_srvRequest()
            request.hand = "both"
            request.state = "open"
            self.open_close_hand_proxy(request)
  
    def callback_get_angles(self, msg):
        if self.grabando:
            if len(self.angles)>0:
                if self.angles[-1] != msg.angles:
                    self.angles.append(msg.angles)
            else:
                self.angles.append(msg.angles)
  
    # --------------- MACHINE INITIALIZATION FUNCTION ---------------            
    def run(self):
        while not rospy.is_shutdown():
            self.zero()

    # --------------- CHECK ROSPY FUNCTION DEFINITION --------------- 
    def check_rospy(self):
        
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            
        print(self.consoleFormatter.format("Shutting down", "FAIL"))
        os._exit(os.EX_OK)

# --------------- INITIALIZATION OF THE TASK CLASS CONSTRUCTOR --------------- 
if __name__ == "__main__":
    sm = CARRY_MY_LUGGAGE()
    sm.run()
    rospy.spin()