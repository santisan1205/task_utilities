#!/usr/bin/env python3
import rospy
import time
import rospkg
import rosservice
import ConsoleFormatter
import json
import math

from std_msgs.msg import Int32, String, Bool
from std_srvs.srv import Trigger, TriggerRequest, SetBool
from geometry_msgs.msg import Twist
from threading import Thread

# All imports from tools

from robot_toolkit_msgs.srv import set_stiffnesses_srv, set_stiffnesses_srvRequest, set_open_close_hand_srv, set_open_close_hand_srvRequest, motion_tools_srv, set_output_volume_srv, set_move_arms_enabled_srv,  misc_tools_srv, misc_tools_srvRequest, tablet_service_srv, battery_service_srv , set_security_distance_srv, move_head_srv, go_to_posture_srv, set_security_distance_srv,set_speechrecognition_srv,speech_recognition_srv, tablet_service_srvRequest, navigate_to_srv, navigate_to_srvRequest, set_angle_srv, set_angle_srvRequest
from robot_toolkit_msgs.msg import touch_msg, speech_recognition_status_msg, set_angles_msg, animation_msg, motion_tools_msg, leds_parameters_msg

from manipulation_msgs.srv import go_to_pose, move_head

from speech_msgs.srv import q_a_srv, talk_srv, speech2text_srv , talk_srvRequest, speech2text_srvRequest, answer_srv, hot_word_srv

from perception_msgs.srv import img_description_srv, get_first_clothes_color_srv, get_clothes_color_srv, start_recognition_srv, get_labels_srv, start_recognition_srvRequest, look_for_object_srv, look_for_object_srvRequest, save_face_srv,save_face_srvRequest, recognize_face_srv, recognize_face_srvRequest, save_image_srv,save_image_srvRequest, set_model_recognition_srv,set_model_recognition_srvRequest,read_qr_srv,read_qr_srvRequest, filter_labels_by_distance_srv, filter_labels_by_distance_srvRequest,turn_camera_srv,turn_camera_srvRequest,filtered_image_srv,filtered_image_srvRequest,start_pose_recognition_srv, get_person_description_srv, add_recognition_model_srv, add_recognition_model_srvRequest, remove_recognition_model_srv, remove_recognition_model_srvRequest, remove_faces_data_srv, calculate_depth_of_label_srv

from navigation_msgs.srv import set_current_place_srv, set_current_place_srvRequest, go_to_relative_point_srv, go_to_relative_point_srvRequest, go_to_place_srv, go_to_place_srvRequest, start_random_navigation_srv, start_random_navigation_srvRequest, add_place_srv, add_place_srvRequest, add_place_with_coordinates_srv ,add_place_with_coordinates_srvRequest,follow_you_srv, follow_you_srvRequest, robot_stop_srv, robot_stop_srvRequest, spin_srv, spin_srvRequest, go_to_defined_angle_srv, go_to_defined_angle_srvRequest, get_absolute_position_srv, get_absolute_position_srvRequest, get_route_guidance_srv, get_route_guidance_srvRequest, correct_position_srv, correct_position_srvRequest, constant_spin_srv, constant_spin_srvRequest
from navigation_msgs.msg import simple_feedback_msg
from perception_msgs.msg import get_labels_msg, get_clothes_color_msg

class Task_module:
    def __init__(
        self,
        perception=False,
        speech=False,
        manipulation=False,
        navigation=False,
        pytoolkit=False,
    ):
        """Initializer for the Task_module class

        Args:
            perception (bool): Enables or disables perception services
            speech (bool): Enables or disables speech services
            manipulation (bool): Enables or disables manipulation services
            navigation (bool): Enables or disables navigation services
        """

        self.consoleFormatter = ConsoleFormatter.ConsoleFormatter()
        ################### GLOBAL VARIABLES ###################
        self.follow_you_active = False
        self.person_attributes = {}
        self.frame_width = 0
        self.frame_height = 0
        self.resolutions_dict = {
            0: (160, 120),
            1: (320, 240),
            2: (640, 480),
            3: (1280, 960),
            4: (2560, 1920)
        }
        self.last_place = ""
        self.current_place = ""
        self.angles = 0
        self.navigating = False
        self.stopped_for_safety = False
        self.avoiding_obstacle = False
        self.center_active = False
        self.head_thread = False
        self.posture_thread = False
        self.linear_vel = 0
        self.closest_person = [0,0,0,0,0]
        self.closest_label = [0,0,0,0,0]
        self.labels = dict()
        self.clothes_color = "pink"
        self.pointing = "none"
        self.hand_up = "none"
        self.say_go_ahead = True
        self.object_found = False
        self.iterationssTouched = False
        self.navigation_status = 0
        self.perception = perception
        self.waiting_head_touch = False
        self.waiting_left_touch = False
        self.waiting_right_touch = False
        self.head_touched = False
        self.hand_touched = False
        self.left_hand_touched = False
        self.right_hand_touched = False
        rospy.init_node("task_utilities")
        self.robot_name = rospy.get_param("/task_utilities/robot_name","nova")
        print("robot name IS",self.robot_name)
        if perception:
            print(
                self.consoleFormatter.format(
                    "Waiting for PERCEPTION services...", "WARNING"
                )
            )
            
            print(
                self.consoleFormatter.format(
                    "Waiting for perception_utilities/remove_faces_data...", "WARNING"
                )
            )
            rospy.wait_for_service("/perception_utilities/remove_faces_data_srv")
            self.remove_faces_data_proxy = rospy.ServiceProxy(
                "/perception_utilities/remove_faces_data_srv", remove_faces_data_srv
            )

            print(
                self.consoleFormatter.format(
                    "Waiting for perception_utilities/turn_camera...", "WARNING"
                )
            )
            rospy.wait_for_service("/perception_utilities/turn_camera_srv")
            self.turn_camera_proxy = rospy.ServiceProxy(
                "/perception_utilities/turn_camera_srv", turn_camera_srv
            )
            
            rospy.wait_for_service("/perception_utilities/filter_labels_by_distance_srv")
            self.filter_by_distance_proxy= rospy.ServiceProxy(
                "/perception_utilities/filter_labels_by_distance_srv", filter_labels_by_distance_srv
            )


            print(
                self.consoleFormatter.format(
                    "Waiting for perception_utilities/get_labels...", "WARNING"
                )
            )
            rospy.wait_for_service("/perception_utilities/get_labels_srv")
            self.get_labels_proxy = rospy.ServiceProxy(
                "/perception_utilities/get_labels_srv", get_labels_srv
            )
            
            print(
                self.consoleFormatter.format(
                    "Waiting for perception_utilities/calculate_depth_of_label...", "WARNING"
                )
            )
            rospy.wait_for_service("/perception_utilities/calculate_depth_of_label_srv")
            self.calculate_depth_of_label_proxy = rospy.ServiceProxy(
                "/perception_utilities/calculate_depth_of_label_srv", calculate_depth_of_label_srv
            )
            
            print(
                self.consoleFormatter.format(
                    "Waiting for perception_utilities/get_clothes_color...", "WARNING"
                )
            )
            rospy.wait_for_service("/perception_utilities/get_clothes_color_srv")
            self.get_clothes_color_proxy = rospy.ServiceProxy(
                "/perception_utilities/get_clothes_color_srv", get_clothes_color_srv
            )

            print(
                self.consoleFormatter.format(
                    "Waiting for perception_utilities/get_first_clothes_color...", "WARNING"
                )
            )
            rospy.wait_for_service("/perception_utilities/get_first_clothes_color_srv")
            self.get_first_clothes_color_proxy = rospy.ServiceProxy(
                "/perception_utilities/get_first_clothes_color_srv", get_first_clothes_color_srv
            )

            print(
                self.consoleFormatter.format(
                    "Waiting for perception_utilities/start_recognition...", "WARNING"
                )
            )
            rospy.wait_for_service("/perception_utilities/start_recognition_srv")
            self.start_recognition_proxy = rospy.ServiceProxy(
                "/perception_utilities/start_recognition_srv", start_recognition_srv
            )

            print(
                self.consoleFormatter.format(
                    "Waiting for perception_utilities/look_for_object...", "WARNING"
                )
            )
            rospy.wait_for_service("/perception_utilities/look_for_object_srv")
            self.look_for_object_proxy = rospy.ServiceProxy(
                "/perception_utilities/look_for_object_srv", look_for_object_srv
            )
            
            print(
                self.consoleFormatter.format(
                    "Waiting for perception_utilities/img_description_with_gpt_vision...", "WARNING"
                )
            )
            rospy.wait_for_service("perception_utilities/img_description_with_gpt_vision_srv")
            self.img_description_proxy = rospy.ServiceProxy(
                "perception_utilities/img_description_with_gpt_vision_srv", img_description_with_gpt_vision_srv
            )

            print(
                self.consoleFormatter.format(
                    "Waiting for perception_utilities/save_face...", "WARNING"
                )
            )
            rospy.wait_for_service("/perception_utilities/save_face_srv")
            self.save_face_proxy = rospy.ServiceProxy(
                "/perception_utilities/save_face_srv", save_face_srv
            )

            print(
                self.consoleFormatter.format(
                    "Waiting for perception_utilities/recognize_face...", "WARNING"
                )
            )
            rospy.wait_for_service("/perception_utilities/recognize_face_srv")
            self.recognize_face_proxy = rospy.ServiceProxy(
                "/perception_utilities/recognize_face_srv", recognize_face_srv
            )
            print(
                self.consoleFormatter.format(
                    "Waiting for perception_utilities/read_qr...", "WARNING"
                )
            )
            rospy.wait_for_service("perception_utilities/read_qr_srv")
            self.qr_read_proxy = rospy.ServiceProxy(
                "/perception_utilities/read_qr_srv", read_qr_srv
            )

            print(
                self.consoleFormatter.format(
                    "Waiting for perception_utilities/filtered_image...", "WARNING"
                )
            )
            rospy.wait_for_service("perception_utilities/filtered_image")
            self.filtered_image_proxy = rospy.ServiceProxy(
                "perception_utilities/filtered_image", filtered_image_srv
            )
            

            print(self.consoleFormatter.format("Waiting for perception_utilities/get_person_description...", "WARNING"))
            rospy.wait_for_service("/perception_utilities/get_person_description_srv")
            self.get_person_description_proxy = rospy.ServiceProxy("/perception_utilities/get_person_description_srv", get_person_description_srv)

            print(
                self.consoleFormatter.format(
                    "Waiting for perception_utilities/pose_srv...", "WARNING"
                )
            )
            rospy.wait_for_service("/perception_utilities/pose_srv")
            self.pose_srv_proxy = rospy.ServiceProxy(
                "/perception_utilities/pose_srv", start_pose_recognition_srv
            )

            print(
                self.consoleFormatter.format(
                    "Waiting for perception_utilities/set_model_recognition...",
                    "WARNING",
                )
            )
            rospy.wait_for_service("perception_utilities/set_model_recognition_srv")
            self.set_model_proxy = rospy.ServiceProxy(
                "perception_utilities/set_model_recognition_srv",
                set_model_recognition_srv,
            )
            
            rospy.wait_for_service("/perception_utilities/add_recognition_model_srv")
            self.add_recognition_model_proxy = rospy.ServiceProxy("/perception_utilities/add_recognition_model_srv", add_recognition_model_srv)
            
            rospy.wait_for_service("/perception_utilities/remove_recognition_model_srv")
            self.remove_recognition_model_proxy = rospy.ServiceProxy("/perception_utilities/remove_recognition_model_srv", remove_recognition_model_srv)

            print(
                self.consoleFormatter.format("PERCEPTION services enabled", "OKGREEN")
            )
            
            self.get_labels_publisher = rospy.Subscriber('/perception_utilities/get_labels_publisher', get_labels_msg, self.callback_get_labels_subscriber)

        self.speech = speech

        if speech:
            print(self.consoleFormatter.format("Waiting for SPEECH services...","WARNING"))

            print(self.consoleFormatter.format("Waiting for /speech_utilities/talk_srv...", "WARNING"))
            rospy.wait_for_service('/speech_utilities/talk_srv')
            self.talk_proxy = rospy.ServiceProxy('/speech_utilities/talk_srv', talk_srv)

            print(self.consoleFormatter.format("Waiting for speech_utilities/speech2text...", "WARNING"))
            rospy.wait_for_service('speech_utilities/speech2text_srv')
            self.speech2text_srv_proxy = rospy.ServiceProxy('speech_utilities/speech2text_srv', speech2text_srv)

            print(self.consoleFormatter.format("Waiting for /speech_utilities/q_a_srv...", "WARNING"))
            rospy.wait_for_service('/speech_utilities/q_a_srv')
            self.q_a_proxy = rospy.ServiceProxy('/speech_utilities/q_a_srv', q_a_srv)

            print(self.consoleFormatter.format("Waiting for speech_utilities/answer...", "WARNING"))
            rospy.wait_for_service('/speech_utilities/answers_srv')
            self.answer_proxy = rospy.ServiceProxy('/speech_utilities/answers_srv', answer_srv)

            print(self.consoleFormatter.format('Waiting for speech_utilities/hot_word_srv service!', 'WARNING'))  
            rospy.wait_for_service('/speech_utilities/hot_word_srv')
            self.hot_word_srv= rospy.ServiceProxy("/speech_utilities/hot_word_srv", hot_word_srv)
            print(self.consoleFormatter.format("SPEECH services enabled","OKGREEN"))


        self.navigation = navigation
        if navigation:
            print(
                self.consoleFormatter.format(
                    "Waiting for NAVIGATION services...", "WARNING"
                )
            )

            print(
                self.consoleFormatter.format(
                    "Waiting for navigation_utilities/set_current_place...", "WARNING"
                )
            )
            rospy.wait_for_service("/navigation_utilities/set_current_place_srv")
            self.set_current_place_proxy = rospy.ServiceProxy(
                "/navigation_utilities/set_current_place_srv", set_current_place_srv
            )

            print(self.consoleFormatter.format("Waiting for navigation_utilities/get_absolute_position_srv...", "WARNING"))
            rospy.wait_for_service("navigation_utilities/get_absolute_position_srv")
            self.get_absolute_position_proxy = rospy.ServiceProxy("navigation_utilities/get_absolute_position_srv",get_absolute_position_srv)

            print(
                self.consoleFormatter.format(
                    "Waiting for navigation_utilities/go_to_relative_point...",
                    "WARNING",
                )
            )
            rospy.wait_for_service("navigation_utilities/go_to_relative_point_srv")
            self.go_to_relative_point_proxy = rospy.ServiceProxy(
                "/navigation_utilities/go_to_relative_point_srv",
                go_to_relative_point_srv,
            )

            print(
                self.consoleFormatter.format(
                    "Waiting for navigation_utilities/go_to_place...", "WARNING"
                )
            )
            rospy.wait_for_service("navigation_utilities/go_to_place_srv")
            self.go_to_place_proxy = rospy.ServiceProxy(
                "/navigation_utilities/go_to_place_srv", go_to_place_srv
            )

            print(
                self.consoleFormatter.format(
                    "Waiting for navigation_utilities/start_random_navigation...",
                    "WARNING",
                )
            )
            rospy.wait_for_service("navigation_utilities/start_random_navigation_srv")
            self.start_random_navigation_proxy = rospy.ServiceProxy(
                "/navigation_utilities/start_random_navigation_srv",
                start_random_navigation_srv,
            )

            print(
                self.consoleFormatter.format(
                    "Waiting for navigation_utilities/add_place...", "WARNING"
                )
            )
            rospy.wait_for_service("navigation_utilities/add_place_srv")
            self.add_place_proxy = rospy.ServiceProxy(
                "/navigation_utilities/add_place_srv", add_place_srv
            )
            rospy.wait_for_service("navigation_utilities/add_place_with_coordinates_srv")
            self.add_place_with_coordinates_proxy = rospy.ServiceProxy(
                "/navigation_utilities/add_place_with_coordinates_srv", add_place_with_coordinates_srv
            )

            print(
                self.consoleFormatter.format(
                    "Waiting for navigation_utilities/follow...", "WARNING"
                )
            )
            rospy.wait_for_service("/navigation_utilities/follow_you_srv")
            self.follow_you_proxy = rospy.ServiceProxy(
                "/navigation_utilities/follow_you_srv", follow_you_srv
            )

            print(
                self.consoleFormatter.format(
                    "Waiting for navigation_utilities/robot_stop_srv...", "WARNING"
                )
            )
            rospy.wait_for_service("navigation_utilities/robot_stop_srv")
            self.robot_stop_proxy = rospy.ServiceProxy(
                "/navigation_utilities/robot_stop_srv", robot_stop_srv
            )

            print(
                self.consoleFormatter.format(
                    "Waiting for navigation_utilities/spin_srv...", "WARNING"
                )
            )
            rospy.wait_for_service("navigation_utilities/spin_srv")
            self.spin_proxy = rospy.ServiceProxy(
                "/navigation_utilities/spin_srv", spin_srv
            )

            print(self.consoleFormatter.format("Waiting for navigation_utilities/go_to_defined_angle_srv...", "WARNING",) )
            rospy.wait_for_service("/navigation_utilities/go_to_defined_angle_srv")
            self.go_to_defined_angle_proxy = rospy.ServiceProxy(    
                "/navigation_utilities/go_to_defined_angle_srv", go_to_defined_angle_srv
            )

            print(
                self.consoleFormatter.format(
                    "Waiting for navigation_utilities/get_route_guidance_srv...",
                    "WARNING",
                )
            )
            rospy.wait_for_service("navigation_utilities/get_route_guidance_srv")
            self.get_route_guidance_proxy = rospy.ServiceProxy(
                "/navigation_utilities/get_route_guidance_srv", get_route_guidance_srv
            )

            print(
                self.consoleFormatter.format(
                    "Waiting for navigation_utilities/constant_spin_srv...", "WARNING"
                )
            )
            rospy.wait_for_service("/navigation_utilities/constant_spin_srv")
            self.constant_spin_proxy = rospy.ServiceProxy(
                "/navigation_utilities/constant_spin_srv", constant_spin_srv
            )

            print(
                self.consoleFormatter.format("NAVIGATION services enabled", "OKGREEN")
            )

        self.manipulation = manipulation
        if manipulation:
            print(
                self.consoleFormatter.format(
                    "Waiting for MANIPULATION services...", "WARNING"
                )
            )

            print(
                self.consoleFormatter.format(
                    "Waiting for manipulation_utilitites/go_to_pose", "WARNING"
                )
            )
            rospy.wait_for_service("manipulation_utilities/go_to_pose")
            self.go_to_pose_proxy = rospy.ServiceProxy(
                "manipulation_utilities/go_to_pose", go_to_pose
            )
            
            print(
                self.consoleFormatter.format(
                    "Waiting for manipulation_utilitites/move_head", "WARNING"
                )
            )
            rospy.wait_for_service("manipulation_utilities/move_head")
            self.move_head_proxy = rospy.ServiceProxy(
                "manipulation_utilities/move_head", move_head
            )

            print(
                self.consoleFormatter.format("MANIPULATION services enabled", "OKGREEN")
            )

        self.pytoolkit = pytoolkit
        if pytoolkit:
            print(
                self.consoleFormatter.format(
                    "Waiting for PYTOOLKIT services...", "WARNING"
                )
            )
            
            print(self.consoleFormatter.format("Waiting for pytoolkit/ALMotion/set_stiffnesses_srv...", "WARNING"))
            rospy.wait_for_service("pytoolkit/ALMotion/set_stiffnesses_srv")
            self.motion_set_stiffnesses_srv = rospy.ServiceProxy("pytoolkit/ALMotion/set_stiffnesses_srv", set_stiffnesses_srv)
            

            print(self.consoleFormatter.format("Waiting for pytoolkit/ALMotion/set_arms_security_srv...", "WARNING"))
            rospy.wait_for_service("pytoolkit/ALMotion/set_arms_security_srv")
            self.set_arms_security = rospy.ServiceProxy("pytoolkit/ALMotion/set_arms_security_srv", SetBool)
            
            
            print(self.consoleFormatter.format("Waiting for pytoolkit/ALAudioDevice/set_output_volume_srv...", "WARNING"))
            rospy.wait_for_service("/pytoolkit/ALAudioDevice/set_output_volume_srv")
            self.set_volume = rospy.ServiceProxy("/pytoolkit/ALAudioDevice/set_output_volume_srv", set_output_volume_srv)
            
            if self.robot_name != "orion":
                
                print(
                    self.consoleFormatter.format(
                        "Waiting for /pytoolkit/ALTabletService/show_web_view_srv...",
                        "WARNING",
                    )
                )
                rospy.wait_for_service("/pytoolkit/ALTabletService/show_web_view_srv")
                self.show_web_proxy = rospy.ServiceProxy(
                    "/pytoolkit/ALTabletService/show_web_view_srv", tablet_service_srv
                )
            
                print(
                    self.consoleFormatter.format(
                        "Waiting for /pytoolkit/ALTabletService/show_image_srv...",
                        "WARNING",
                    )
                )
                rospy.wait_for_service("/pytoolkit/ALTabletService/show_image_srv")
                self.show_image_proxy = rospy.ServiceProxy(
                    "/pytoolkit/ALTabletService/show_image_srv", tablet_service_srv
                )
                
                
                print(self.consoleFormatter.format("Waiting for /pytoolkit/ALTabletService/hide_srv...", "WARNING"))
                rospy.wait_for_service("/pytoolkit/ALTabletService/hide_srv")
                self.hide_proxy = rospy.ServiceProxy("/pytoolkit/ALTabletService/hide_srv",battery_service_srv)
                
                print(self.consoleFormatter.format("Waiting for /pytoolkit/ALTabletService/play_video_srv...", "WARNING"))
                rospy.wait_for_service("/pytoolkit/ALTabletService/play_video_srv")
                self.show_video_proxy = rospy.ServiceProxy("/pytoolkit/ALTabletService/play_video_srv",tablet_service_srv)

                print(self.consoleFormatter.format("Waiting for pytoolkit/show_topic...", "WARNING"))
                rospy.wait_for_service("/pytoolkit/ALTabletService/show_topic_srv")
                self.show_topic_proxy = rospy.ServiceProxy("/pytoolkit/ALTabletService/show_topic_srv", tablet_service_srv)
                print(
                self.consoleFormatter.format(
                    "Waiting for pytoolkit/show_words...", "WARNING"
                )
                )
                rospy.wait_for_service("/pytoolkit/ALTabletService/show_words_srv")
                self.show_words_proxy = rospy.ServiceProxy(
                    "/pytoolkit/ALTabletService/show_words_srv", battery_service_srv
                )
            
            
                print(self.consoleFormatter.format("Waiting for pytoolkit/ALTabletService/show_picture_srv...", "WARNING"))
                rospy.wait_for_service('pytoolkit/ALTabletService/show_picture_srv')
                self.show_picture_proxy = rospy.ServiceProxy('pytoolkit/ALTabletService/show_picture_srv', battery_service_srv)
            
            print(
                self.consoleFormatter.format(
                    "Waiting for /pytoolkit/ALSpeechRecognition/set_hot_word_language_srv...",
                    "WARNING",
                )
            )
            rospy.wait_for_service('/pytoolkit/ALSpeechRecognition/set_hot_word_language_srv')
            self.hot_word_language_srv = rospy.ServiceProxy("/pytoolkit/ALSpeechRecognition/set_hot_word_language_srv", tablet_service_srv)

            self.setMoveHead_srv = rospy.ServiceProxy(
                "/pytoolkit/ALMotion/move_head_srv",
                move_head_srv
            )

            self.toggle_get_angles_topic_srv = rospy.ServiceProxy(
                "/pytoolkit/ALMotion/toggle_get_angle_srv",
                set_angle_srv
            )

            self.set_angles_proxy = rospy.ServiceProxy(
                "/pytoolkit/ALMotion/set_angle_srv",
                set_angle_srv
            )

            self.Navigate_to_srv = rospy.ServiceProxy(
                "/pytoolkit/ALNavigation/navigate_to_srv",
                navigate_to_srv
            )
            
            self.enable_security_proxy = rospy.ServiceProxy(
                "/pytoolkit/ALMotion/enable_security_srv",
                battery_service_srv
            )
            
            self.setDistance_srv = rospy.ServiceProxy(
                "/pytoolkit/ALMotion/set_security_distance_srv",
                set_security_distance_srv
            )

            self.setRPosture_srv = rospy.ServiceProxy(
                "/pytoolkit/ALRobotPosture/go_to_posture_srv",
                go_to_posture_srv
            )

            self.setMoveArms_srv = rospy.ServiceProxy(
                "pytoolkit/ALMotion/set_move_arms_enabled_srv",
                set_move_arms_enabled_srv
            )
            print(
                self.consoleFormatter.format(
                    "Waiting for pytoolkit/awareness...", "WARNING"
                )
            )
            rospy.wait_for_service("/pytoolkit/ALBasicAwareness/set_awareness_srv")
            self.awareness_proxy = rospy.ServiceProxy(
                "/pytoolkit/ALBasicAwareness/set_awareness_srv", SetBool
            )
            
            print(self.consoleFormatter.format("Waiting for pytoolkit/ALMotion/set_orthogonal_security_distance_srv...", "WARNING"))
            rospy.wait_for_service("/pytoolkit/ALMotion/set_orthogonal_security_distance_srv")
            self.set_orthogonal_security_srv = rospy.ServiceProxy("/pytoolkit/ALMotion/set_orthogonal_security_distance_srv",set_security_distance_srv)

            print(self.consoleFormatter.format("Waiting for pytoolkit/ALMotion/set_tangential_security_distance_srv...", "WARNING"))
            rospy.wait_for_service("/pytoolkit/ALMotion/set_tangential_security_distance_srv")
            self.set_tangential_security_srv = rospy.ServiceProxy("/pytoolkit/ALMotion/set_tangential_security_distance_srv",set_security_distance_srv)
            
            rospy.wait_for_service("/pytoolkit/ALMotion/set_security_distance_srv")
            self.set_security_distance_proxy = rospy.ServiceProxy("/pytoolkit/ALMotion/set_security_distance_srv", set_security_distance_srv)
            print(self.consoleFormatter.format("Waiting for pytoolkit/ALMotion/set_security_distance_srv...", "WARNING"))

            print(self.consoleFormatter.format("Waiting for pytoolkit/autonomousLife...", "WARNING"))
            rospy.wait_for_service("/pytoolkit/ALAutonomousLife/set_state_srv")
            self.autonomous_life_proxy = rospy.ServiceProxy("/pytoolkit/ALAutonomousLife/set_state_srv",SetBool)

            print(self.consoleFormatter.format("Waiting for pytoolkit/stop_tracker...", "WARNING"))
            rospy.wait_for_service("/pytoolkit/ALTracker/stop_tracker_srv")
            self.stop_tracker_proxy = rospy.ServiceProxy("/pytoolkit/ALTracker/stop_tracker_srv",battery_service_srv)

            print(self.consoleFormatter.format("Waiting for pytoolkit/start_tracker...", "WARNING"))
            rospy.wait_for_service("/pytoolkit/ALTracker/start_tracker_srv")
            self.start_tracker_proxy = rospy.ServiceProxy("/pytoolkit/ALTracker/start_tracker_srv",battery_service_srv)

            print(self.consoleFormatter.format("Waiting for pytoolkit/start_follow_face...", "WARNING"))
            rospy.wait_for_service("/pytoolkit/ALTracker/start_follow_face")
            self.start_follow_face_proxy = rospy.ServiceProxy("/pytoolkit/ALTracker/start_follow_face",battery_service_srv)
            
            print(self.consoleFormatter.format("Waiting for /pytoolkit/ALMotion/toggle_breathing_srv...", "WARNING"))
            rospy.wait_for_service("/pytoolkit/ALMotion/toggle_breathing_srv")
            self.toggle_breathing_proxy = rospy.ServiceProxy("/pytoolkit/ALMotion/toggle_breathing_srv", set_open_close_hand_srv)
        
            # PYTOOLKIT Suscribers
            
            self.move_publisher = rospy.Publisher('/pytoolkit/ALMotion/move', Twist, queue_size=10)
            self.subscriber_angles = rospy.Subscriber("/pytoolkit/ALMotion/get_angles",set_angles_msg,self.callback_get_angles)
            
            
            
            print(self.consoleFormatter.format("PYTOOLKIT services enabled", "OKGREEN"))
           
        # SUBSCRIBERS
        self.animationPublisher = rospy.Publisher('/animations', animation_msg, queue_size=10)
            
        self.ledsPublisher = rospy.Publisher('/leds', leds_parameters_msg, queue_size=10)
        
        self.SensorSubscriber = rospy.Subscriber("/touch", touch_msg, self.callback_sensor_subscriber)

    #################################### SERVICES #######################################

    def initialize_node(self, task_name):
        """
        Initialized the node with the name of the task
        """
        pass
        

    def initialize_pepper(self):
        """
        Initializes the pep robot with default parameteres
        """
        if self.perception:
            self.turn_camera("front_camera","custom",1,20)
            self.start_recognition("front_camera")
        else:
            print("perception as false")
        if self.pytoolkit:
            if self.robot_name == "nova" or self.robot_name == "opera":
                self.enable_security_proxy()
                self.show_words_proxy()
                self.setRPosture_srv("stand")
                self.set_orthogonal_security_srv(0.3)
                self.set_tangential_security_srv(0.05)
            elif self.robot_name == "orion":
                self.setRPosture_srv("stand")
        else:
            print("pytoolkit as false")

    ################### PERCEPTION SERVICES ###################

    def turn_camera(
        self, camera_name: str, command="custom", resolution=1, fps=15
    ) -> bool:
        """
        Input:
        camera_name: "front_camera" || "bottom_camera" || "depth_camera"
        command: "enable" || "disable" || "custom"
        resolution: RGB (0-4) DEPTH (0-1)
        fps: 1-30
        Output: True if the service was called correctly, False if not
        ----------
        Turns on/off camera_name, if command is custom, it sets the resolution and fps
        """
        if self.perception:
            self.frame_width, self.frame_height = self.resolutions_dict[resolution]
            try:
                approved = self.turn_camera_proxy(camera_name, command, resolution, fps)
                if approved == "approved":
                    return True
                else:
                    return False
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("perception as false")
            return False

    def publish_filtered_image(self, filter_name: str, camera_name: str) -> bool:
        """
        Input:
        filter_name: "face" || "qr"
        camera_name: "front_camera" || "bottom_camera"
        Output: True if the service was called correctly, False if not
        ----------
        Publishes the filtered image from camera_name with filter_name
        """
        if self.perception:
            try:
                approved = self.filtered_image_proxy(filter_name)
                return approved.approved
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("perception as false")
            return False

    def start_recognition(self, camera_name: str) -> bool:
        """
        Input:
        camera_name: "front_camera" || "bottom_camera"
        Output: True if the service was called correctly, False if not
        ----------
        Starts recognition for <camera_name>, if <camera_name> is empty, it shuts down all recognition
        """
        if self.perception:
            try:
                approved = self.start_recognition_proxy(camera_name)
                if approved == "approved":
                    return True
                else:
                    return False
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("perception as false")
            return False

    def get_labels(self, bool):
        if self.perception:
            try:
                return self.get_labels_proxy.call(bool)
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
            return ""
        
    def calculate_depth_of_label(self, x, y, w, h):
        """
        Input:
        x, y, w, h: x, y and width and height of label
        Output: float: distance in meters of the label corresponding to the given coordinates
        """
        if self.perception:
            try:
                return self.calculate_depth_of_label_proxy(x, y, w, h).depth
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return 0
        else:
            print("perception as false")
            return 0
            
    def look_for_object(self, object_name: str, ignore_already_seen=False) -> bool:
        """
        Input:
        object_name: label of the object to look for options -> classes names depends of the actual model (see set_model service)
        ignore_already_seen: True->ignore objects already seen || False->don't ignore objects already seen
        Output: True if the service was called correctly, False if not
        ----------
        Looks for object_name in the current get_labels topic, publishes T/F in look_for_object_publisher
        """
        if self.perception:
            try:
                approved = self.look_for_object_proxy(object_name, ignore_already_seen)
                if approved == "approved":
                    return True
                else:
                    return False
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("perception as false")
            return False

    def get_all_items(self, place="") -> list:
        """
        Input:
        place: The specific place the robot is looking for the item.
        Output: List of items the robot saw
        ----------
        Aks chatgpt vision for the items in front of the robot
        """
        if self.perception:
            try:
                angles_to_check = [0]
                self.setRPosture_srv("stand")
                for angle in angles_to_check:
                    print("angulo actual:",angle)
                    self.set_angles_srv(["HeadYaw","HeadPitch"],[math.radians(angle), 0],0.1)
                    if angle==0:
                        rospy.sleep(1)
                    elif angle==-60:
                        rospy.sleep(3)
                    elif angle==60:
                        rospy.sleep(5)
                    place_prompt = ""
                    if place != "":
                        place_prompt = f" in the {place}"
                    gpt_vision_prompt = f"Answer in a comma separated string (ie: 'bowl,cup,bottle'): give me a list of all the items you see{place_prompt} in the image."
                    answer = self.img_description(gpt_vision_prompt,camera_name="both")["message"]
                    if not "none" in answer.lower():
                        break
                self.setRPosture_srv("stand")
                return answer.split(",")
                
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return "None"
        else:
            print("perception as false")
            return "error"

    def find_item_with_characteristic(self, class_type,characteristic,place="") -> str:
        """
        Input:
        class_type: The characteristic to search for ("color", "size", "weight", "position", "description")
        characteristic: The specific thing to search for (example: red,blue,black, white dots, smallest, largest, thinnest, big one, lightest, heaviest,left, right,fragile)
        place: The specific place the robot is looking for the item.
        Output: String with the item with the characteristic name or "none" if it wasn't found
        ----------
        Aks chatgpt vision which item in front of the robot has the characteristic
        """
        if self.perception:
            try:
                angles_to_check = [0]
                self.setRPosture_srv("stand")
                found = False
                for angle in angles_to_check:
                    print("angulo actual:",angle)
                    self.set_angles_srv(["HeadYaw","HeadPitch"],[math.radians(angle), 0],0.1)
                    if angle==0:
                        rospy.sleep(1)
                    elif angle==-60:
                        rospy.sleep(3)
                    elif angle==60:
                        rospy.sleep(5)
                    place_prompt = ""
                    if place != "":
                        place_prompt = f" in the {place}"
                    if class_type == "color":
                        gpt_vision_prompt = f"Which item{place_prompt} shown in the picture has these colors: {characteristic}. Answer only with one word, either the item name or None"
                    elif class_type == "size" or class_type == "weight":
                        gpt_vision_prompt = f"Which item{place_prompt} shown in the picture is the {characteristic}. Answer only with one word, either the item name or None"
                    elif class_type == "position":
                        gpt_vision_prompt = f"Which item{place_prompt} shown in the picture is positioned {characteristic}most. Answer only with one word, either the item name or None"
                    elif class_type == "description":
                        gpt_vision_prompt = f"Which item{place_prompt} shown in the picture is {characteristic}. Answer only with one word, either the item name or None"
                    answer = self.img_description(gpt_vision_prompt,camera_name="bottom_camera")["message"]
                    if not "none" in answer.lower():
                        break
                self.setRPosture_srv("stand")
                return answer
                
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return "None"
        else:
            print("perception as false")
            return "error"

    def get_person_gesture(self)->str:
        """
        Input:
        Output: True if person was found
        ----------
        Looks for a specific person. This person is pointing, raising a hand, has a specific name, is breaking a particular rule or TODO is wearing a color
        """
        if self.perception and self.manipulation:
            self.setRPosture_srv("stand")
            gpt_vision_prompt = f"Answer about the person centered in the image. What gesture is the person doing. Answer only with one word or 'None' if you couldn't determine the gesture"
            answer = self.img_description(gpt_vision_prompt, camera_name="both")["message"]
            return answer
        else:
            print("perception or manipulation as false")
            return "None"

    def wait_for_head_touch(self, timeout = 15, message = "", message_interval = 5,language="English"):
        """
        Input:
        timeout: The time until the robot stops waiting
        message: The message the robot should repeat every x seconds
        message_interval: The time the robot should wait between talks
        Output: True if the robot head was touched, False if timeout
        ----------
        Waits until the robots head is touched or a timeout
        """
        first_time = True
        self.head_touched = False
        self.waiting_head_touch = True
        start_time = rospy.get_time()
        last_talk_time = rospy.get_time()
        while (not self.head_touched) and rospy.get_time() - start_time < timeout:
            rospy.sleep(0.1)
            if rospy.get_time()-last_talk_time > message_interval and not first_time:
                self.talk(message,language,wait=True)
                last_talk_time = rospy.get_time()
            if first_time:
                first_time = False
        result = self.head_touched
        self.waiting_head_touch = False
        self.head_touched = False
        return result

    def wait_for_arm_touch(self, timeout = 15, message = "", message_interval = 5,language="English",arm="both"):
        """
        Input:
        timeout: The time until the robot stops waiting
        message: The message the robot should repeat every x seconds
        message_interval: The time the robot should wait between talks
        Output: True if the robot arm was touched, False if timeout
        ----------
        Waits until the robots arm is touched or a timeout
        """
        first_time = True
        self.hand_touched = False
        self.left_hand_touched = False
        self.right_hand_touched = False
        start_time = rospy.get_time()
        last_talk_time = rospy.get_time()
        if arm == "both":
            self.waiting_right_touch = True
            self.waiting_left_touch = True
            while (not self.hand_touched) and rospy.get_time() - start_time < timeout:
                rospy.sleep(0.1)
                if rospy.get_time()-last_talk_time > message_interval and not first_time:
                    self.talk(message,language,wait=True)
                    last_talk_time = rospy.get_time()
                if first_time:
                    first_time = False
            result = self.hand_touched
            self.waiting_right_touch = False
            self.waiting_left_touch = False
            self.hand_touched = False
            self.left_hand_touched = False
            self.right_hand_touched = False
            return result
        elif arm == "left":
            self.waiting_left_touch = True
            while (not self.left_hand_touched) and rospy.get_time() - start_time < timeout:
                rospy.sleep(0.1)
                if rospy.get_time()-last_talk_time > message_interval and not first_time:
                    self.talk(message,language,wait=True)
                    last_talk_time = rospy.get_time()
                if first_time:
                    first_time = False
            result = self.left_hand_touched
            self.waiting_left_touch = False
            self.hand_touched = False
            self.left_hand_touched = False
            self.right_hand_touched = False
            return result
            
        elif arm == "right":
            self.waiting_right_touch = True
            while (not self.right_hand_touched) and rospy.get_time() - start_time < timeout:
                rospy.sleep(0.1)
                if rospy.get_time()-last_talk_time > message_interval and not first_time:
                    self.talk(message,language,wait=True)
                    last_talk_time = rospy.get_time()
                if first_time:
                    first_time = False
            result = self.right_hand_touched
            self.waiting_right_touch = False
            self.hand_touched = False
            self.left_hand_touched = False
            self.right_hand_touched = False
            return result

    def search_for_specific_person(self, class_type: str, specific_characteristic: str,true_check=False) -> bool:
        """
        Input:
        class_type: The characteristic to search for (name, pointing, raised_hands or colors)
        specific_characteristic: The specific thing to search for (example: David, Right hand up, Pointing right, blue (shirt,coat,etc))
        Output: True if person was found
        ----------
        Looks for a specific person. This person is pointing, raising a hand, has a specific name, is breaking a particular rule or TODO is wearing a color
        """
        if self.perception and self.manipulation:
            try:
                angles_to_check = [0]
                self.setRPosture_srv("stand")
                self.look_for_object("person", ignore_already_seen=True)
                specific_person_found = False
                for angle in angles_to_check:
                    self.set_angles_srv(["HeadYaw","HeadPitch"],[math.radians(angle), -0.1],0.1)
                    if angle==0:
                        rospy.sleep(1)
                    elif angle==-60:
                        rospy.sleep(3)
                    elif angle==60:
                        rospy.sleep(5)
                    persons = self.labels.get("person", [])
                    for person in persons:
                        self.center_head_with_label(person)
                        if true_check:
                            if class_type=="name":
                                name = self.q_a("name")
                                specific_characteristic = specific_characteristic.lower().replace(".","").replace("!","").replace("?","")
                                if specific_characteristic in name:
                                    specific_person_found = True 
                                    break
                            elif class_type=="pointing":
                                if self.pointing == specific_characteristic:
                                    specific_person_found = True 
                                    break
                            elif class_type=="raised_hand":
                                if self.hand_up == specific_characteristic:
                                    specific_person_found = True
                                    break
                            elif class_type=="colors":
                                gpt_vision_prompt = f"Answer about the person centered in the image: Is the person wearing {specific_characteristic}? Answer only with True or False"
                                answer = self.img_description(gpt_vision_prompt)["message"]
                                if "True" in answer:
                                    specific_person_found = True
                                    break
                        else:
                            specific_person_found = True
                            break
                        # Despues de centrar la cabeza, devolverla al punto inicial para centrar a otra persona
                        self.set_angles_srv(["HeadYaw","HeadPitch"],[math.radians(angle), -0.1],0.1)
                    if specific_person_found:
                        break
                self.setRPosture_srv("stand")
                return specific_person_found
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("perception or manipulation as false")
            return False

    def wait_for_object(self, timeout: float) -> bool:
        """
        Input: timeout in seconds || -1 for infinite
        Output: True->Found || False->Timeout/Fail
        ----------
        Waits for object to be found for a max of <timeout> seconds
        """
        if self.perception:
            try:
                print("Waiting for object")
                t_start = time.time()
                finish = False
                response = False
                subscriber = rospy.Subscriber(
                    "/perception_utilities/look_for_object_publisher",
                    Bool,
                    self.callback_look_for_object,
                )
                while not finish:
                    rospy.sleep(0.0666)
                    t_now = time.time()
                    if self.object_found:
                        finish = True
                        response = True
                    elif (
                        t_now - t_start > timeout or rospy.is_shutdown()
                    ) and timeout > 0:
                        finish = True
                        response = False
                return response
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("perception as false")
            return False

    def find_object(self, object_name: str, timeout=25, ignore_already_seen=False) -> bool:
        """
        Input:
        object_name: label of the object to look for options -> classes names depends of the actual model (see set_model service)
        timeout: time in seconds to look for the object while spinning
        Output: True if the object was found, False if not
        ----------
        Spins while looking for <object_name> for <timeout> seconds while spinning at 15 deg/s
        """
        # spins until the object is found or timeout
        if self.perception and self.navigation and self.manipulation:
            try:
                angles_to_check = [0,-60,60]
                self.setRPosture_srv("stand")
                self.look_for_object(object_name, ignore_already_seen=ignore_already_seen)
                found = False
                for angle in angles_to_check:
                    print("angulo actual:",angle)
                    self.set_angles_srv(["HeadYaw","HeadPitch"],[math.radians(angle), 0],0.1)
                    if angle==0:
                        rospy.sleep(1)
                    elif angle==-60:
                        rospy.sleep(3)
                    elif angle==60:
                        rospy.sleep(5)
                    labels = self.labels.get(object_name, [])
                    if len(labels)>0:
                        label_found = labels[0]
                        self.center_head_with_label(label_found)
                        found = True
                        break
                self.setRPosture_srv("stand")
                return found
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("perception or navigation or manipulation as false")

    def count_objects(self, object_name: str) -> int:
        """
        Input: object_name, classes names depends of the actual model (see set_model service)
        Output: Number of objects seen when rotating 360
        ----------
        Spins 360 degrees and then returns the number of objects of <object_name> seen
        """
        if self.perception and self.navigation and self.manipulation and self.pytoolkit:
            try:
                self.setRPosture_srv("stand")
                angles_to_check = [0]
                gpt_vision_prompt = f"How many {object_name} are there in the picture? Answer only with an Integer number of occurrrences"
                counter = 0
                for angle in angles_to_check:
                    self.set_angles_srv(["HeadYaw","HeadPitch"],[math.radians(angle), -0.1],0.2)
                    if angle==0:
                        rospy.sleep(1)
                    elif angle==-60:
                        rospy.sleep(3)
                    elif angle==60:
                        rospy.sleep(5)
                    answer = self.img_description(gpt_vision_prompt, camera_name="both")["message"]
                    print(answer)
                    if answer.isdigit():
                        counter+= int(answer)
                self.setRPosture_srv("stand")
                return counter
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("perception, manipulation or navigation as false")

    def save_face(self, name: str, num_pics=5) -> bool:
        """
        Input: name, num_pics
        Output: True if the service was called correctly, False if not
        ----------
        Saves num_pics of the face of the person with name and its encodings
        """
        if self.perception:
            try:
                approved = self.save_face_proxy(name, num_pics)
                print("approved", approved)
                return approved.approved
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("perception as false")
            return False

    def recognize_face(self, num_pics=3) -> str:
        """
        Input: num_pics of the face to recognize
        Output: name of the recognized person
        ----------
        Recognizes the face of the person with <num_pics>
        """
        if self.perception:
            try:
                response = self.recognize_face_proxy(num_pics)
                if response.approved:
                    return response.person
                else:
                    return ""
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return ""
        else:
            print("perception as false")
            return ""
    
    def remove_faces_data(self):
        """
        Removes all the faces data from the perception utilities 'resources/data' directory
        """
        if self.perception:
            try:
                approved = self.remove_faces_data_proxy()
                return approved.approved
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("perception as false")
            return False

    def get_person_description(self) -> dict:
        """
        Input:
        Output: dict with the description of the person {"status":(happy,neutral,sad,angry),"gender":("Man","Woman"),"age":(int),"race":(race of the person)}
        ----------
        Returns a dict with the description of the person
        """
        attributes = {}
        if self.perception:
            try:
                response = self.get_person_description_proxy()
                attributes = {
                    "gender": response.gender,
                    "age": int(response.age),
                }
                self.person_attributes = attributes
                return attributes
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return {}
        else:
            print("perception as false")
            return {}
        
    def get_clothes_color(self) -> str:
        """
        Input:
        Output: color of the clothes of the person
        ----------
        Returns the color of the clothes of the person
        """
        if self.perception:    
            try:
                self.get_clothes_color_proxy.call(True)
                time.sleep(0.2)
                response = self.get_first_clothes_color_proxy.call()
                return response.color
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
            return ""
        else:
            print("perception as false")
            return ""
        
    def get_clothes_color_from_publisher(self):
        
        self.get_clothges_color_subscriber = rospy.Subscriber('/perception_utilities/get_clothes_publisher', get_clothes_color_msg, self.callback_get_clothes_color_subscriber)
        print(self.clothes_color)
        return self.clothes_color
        
        
    def img_description(self, prompt: str, camera_name="front_camera", distance=0.0) -> dict:
        """
        Input:
        camera_name: "front_camera" || "bottom_camera" || "depth_camera" || "Both
        prompt: A string that indicates what gpt vision must do with the image. example: "Describe this image:"
        distance: distance threshold if camera_name is "depth_camera"
        Output: Dictionary containing the answer from gpt vision.
        ----------
        Make a call to the gpt vision api with an image of what the robot is currently seeing.
        """
        attributes = {}
        if self.perception:
            try:
                response = self.img_description_proxy(camera_name, prompt, distance)
                attributes = {
                    "status": response.approved,
                    "message": response.message
                }
                return attributes
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return {}
        else:
            print("perception as false")
            return {}
        
        
    def toggle_filter_by_distance(self, activate: bool, distance: float, labels=["person"]) -> dict:
        """
        Input:
        activate: True to filter, false to stop filtering labels by distance
        distance: A float that indicates the distance threshold in meters at which perception utilities should not publish the detected label
        labels: A list of labels to filter by distance
        ----------
        Toggles the filter by distance in perception utilities to stop the node from publishing far away labels
        """
        if self.perception:
            try:
                filter_label_msg = filter_labels_by_distance_srvRequest()
                filter_label_msg.activate = activate
                filter_label_msg.labels = labels
                filter_label_msg.threshold_in_meters = distance
                self.filter_by_distance_proxy(filter_label_msg)
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return {}
        else:
            print("perception as false")
            return {}

                

    def pose_srv(self, camera_name: str, start: bool) -> bool:
        """
        Input: camera_name
        Output: True if the service was called correctly, False if not
        ----------
        Saves the pose of the person in front of the camera
        """
        if self.perception:
            try:
                approved = self.pose_srv_proxy(camera_name, start)
                return approved.approved
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("perception as false")
            return False

    def qr_read(self, timeout: float) -> str:
        """
        Input: timeout in seconds
        Output: "text" read from qr || "" if timeout or fail
        ----------
        Reads a qr code for a max of <timeout> seconds
        """
        if self.perception:
            try:
                read_qr_message = read_qr_srvRequest()
                read_qr_message.timeout = timeout
                text = self.qr_read_proxy(read_qr_message)
                print(text)
                return text.text
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return ""
        else:
            print("perception as false")
            return ""

    def set_model(self, model_name: str) -> bool:
        """
        Input: model name -> "default" || "objects" || "fruits"
        Output: True if the service was called correctly, False if not
        ----------
        Sets the model to use for the object recognition.
        classes by model:
        default: ["person","bench","backpack","handbag","suitcase","bottle","cup","fork","knife","spoon","bowl","chair","couch","bed","laptop"]
        objects: ["spam"(carne enlatada), "cleanser", "sugar", "jello"(gelatina roja), "mug", "tuna", "bowl", "tomato_soup", "footwear", "banana", "mustard", "coffee_grounds", "cheezit"]
        fruits: ["apple", "lemon", "orange", "peach", "pear", "plum","strawberry"]
        """
        if self.perception:
            try:
                approved = self.set_model_proxy(model_name)
                # wait for the model to be set
                rospy.sleep(3)
                return approved.approved
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("perception as false")
            return False
        
    def add_model(self, camera_name: str, model_name: str) -> bool:
        """
        Adds a recognition model to the specified camera.

        Args:
            camera_name (str): The name of the camera to add the model to.
            model_name (str): The name of the model to add.

        Returns:
            bool: True if the model was successfully added, False otherwise.
        ---
        Valid parameter values:
        camera_name: "front_camera" || "bottom_camera"
        model_name: "mercadito" || "comidita"
        """
        
        if self.perception:
            try:
                response = self.add_recognition_model_proxy(camera_name, model_name)
                rospy.sleep(1)
                return response.approved
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("perception as false")
            return False

    def remove_model(self, camera_name: str, model_name: str) -> bool:
        """
        Removes a recognition model from the specified camera. Ideally, the model must already be added to the camera to avoid errors

        Args:
            camera_name (str): The name of the camera to remove the model from.
            model_name (str): The name of the model to remove.

        Returns:
            bool: True if the model was successfully removed, False otherwise.
        ---
        Valid parameter values:
        camera_name: "front_camera" || "bottom_camera"
        model_name: "mercadito" || "comidita"
        """
        if self.perception:
            try:
                response = self.remove_recognition_model_proxy(camera_name, model_name)
                rospy.sleep(1)
                return response.approved
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("perception as false")
            return False
    
    
    def get_closer_label(self, label) -> None:
        """
        Input: None
        Output: None
        ----------
        Updates the information of the person that is closest to the robot.
        Runs in a thread in follow you
        """
        # yolo_awareness_active is used to halt thread execution
        while self.yolo_awareness_active:
            labels_actuales = self.labels
            for clabel in labels_actuales:
                if clabel == label:
                    self.closest_label = max(labels_actuales[clabel], key=lambda x: x[3])
                    self.iterations = 0
    
    def get_closer_person(self) -> None:
        """
        Input: None
        Output: None
        ----------
        Updates the information of the person that is closest to the robot.
        Runs in a thread in follow you
        """
        # follow_you_active is used to halt thread execution
        while self.follow_you_active:
            labels_actuales = self.labels
            for label in labels_actuales:
                if label == "person":
                    self.closest_person = max(labels_actuales[label], key=lambda x: x[3])
                    self.iterations = 0
    
    def get_closest_object(self, object_name:str) -> None:
        """
        Input: (str) object_name
        Output: None
        ----------
        Updates the information of the object that is closest to the robot.
        Runs in a thread.
        """
        # center_active is used to halt thread execution
        while self.center_active:
            labels_actuales = self.labels
            for label in labels_actuales:
                if label == object_name:
                    self.closest_object = max(labels_actuales[label], key=lambda x: x[3])
                    self.iterations = 0

    ################### SPEECH SERVICES ###################

    def talk(self, text: str, language="English", wait=True, animated=False, speed = "100") -> bool:
        """
        Input:
        text
        language: English || Spanish
        wait(wait until the robot finishes talking)
        animates: gesture hands
        Output: True if everything ok || False if not
        ----------
        Allows the robot to say the input of the service.
        """
        if self.speech:
            try:
                self.talk_proxy(text, language, wait, animated, speed)
                return True
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("speech as false")
            return False
        
    def hot_word(self, words: list, noise = False, eyes = False, thresholds = None, language="English"):
        """
        Input:
        words -> list of words to detect
        noise -> True || False if there is a noise when the hot worrd is detected
        eyes -> True || False if the eyes shine 
        threshold -> Threshold to detect a word
        Output: True if everything ok || False if not
        ----------
        Allows the robot to detect hot words
        """
        self.hot_word_language_srv(language)
        if thresholds == None:
            thresholds = [0.4 for _ in range(len(words))]
        else:
            if len(words)!= len(thresholds):
                error_msg = "Words: "+str(len(words))+" Thresholds: "+str(len(thresholds))
                print(self.consoleFormatter.format(error_msg, "FAIL"))
                return False
            
        if self.speech:
            try:
                self.hot_word_srv(words, noise, eyes, thresholds)
                return True
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("speech as false")
            return False

    def speech2text_srv(
        self, seconds=0, lang="eng"
    ) -> str:
        """
        Input:
        seconds: 0 for automatic stop || >0 for seconds to record
        file_name: name of the file
        transcription: True || False
        Output: text that the robot heard
        ----------
        Allows the robot to understand what a person is saying and return a string with the words.
        """
        if self.speech:
            try:
                text = self.speech2text_srv_proxy(seconds, lang)
                return text.transcription
            except rospy.ServiceException as e  :
                print("Service call failed: %s" % e)
                return ""
        else:
            print("speech as false")
            return ""
        
    def live_transcription_proxy_thread(self):
        self.live_transcription_proxy(True)
        rospy.time(1000)
        
    def answer_question(self, question:str, temperature = 0.5, save_conversation = True, fill_time = True)->str:
        """
        Input: 
        question: String question to respond 
        temperature: scale from 0 to 1 being 0 deterministic and 1 creative
        save_conversation: Save history of the conversation or not
        fill_time: Fill with speaking 
        Output: 
        answer: A string with the answer to the question
        """
        if self.speech:
            try:
                response = self.answer_proxy(question,save_conversation, temperature, "").answer
            except rospy.ServiceException as e:
                print("Service call failedL %s"%e)
                response = "I could not find relevant followed_persons for your question"
        return response
    
    def q_a(self, tag:str)->str:
        """
        Input: tag in lowercase: options -> ("age", "name", "drink")
        Output: answer
        ----------
        Returns a specific answer for predefined questions.
        """
        if self.speech:
            try:
                answer = self.q_a_proxy(tag)
                return answer.answer
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return ""
        else:
            print("speech as false")
            return ""

    ################### NAVIGATION SERVICES ###################

    def set_current_place(self, place_name: str) -> bool:
        """
        Input: place_name
        Output: True if the service was called correctly, False if not
        ----------
        Sets the place of the robot to the coordinates of the place specified in
        the request message, then it clears the global costmap.
        """
        if self.navigation:
            try:
                self.set_move_arms_enabled(False)
                approved = self.set_current_place_proxy(place_name)
                self.last_place = self.current_place
                self.current_place = place_name
                if approved == "approved":
                    return True
                else:
                    return False
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("navigation as false")
            return False

    def go_to_relative_point(self, x: float, y: float, theta: float) -> bool:
        """
        Input: x, y, theta
        Output: True if the service was called correctly, False if not
        ----------
        Sends the robot to the coordinates (in meters relative to the robot and rotates theta angle (raidans) relative)
        specified in the request message.
        """
        if self.navigation:
            approved=False
            try:
                self.set_move_arms_enabled(False)
                if self.pytoolkit:
                    approved = self.go_to_relative_point_proxy(x,y,theta)
                if approved=="approved":
                    return True
                else:
                    return False
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("navigation as false")
            return False

    def go_to_place(self, place_name: str, graph=1, wait=True, lower_arms=True) -> bool:
        """
        Input:
        place_name: options -> ("door","living_room")
        graph: 0 no graph || 1 graph
        wait: True (waits until the robot reaches) || False (doesn't wait)
        Output: True if the service was called correctly, False if not
        ----------
        Goes to place_name
        """
        if self.navigation:
            try:
                self.set_move_arms_enabled(False)
                approved = self.go_to_place_proxy(place_name, graph)
                if wait and not "same-place" in approved.answer:
                    self.wait_go_to_place()
                self.last_place = self.current_place
                self.current_place = place_name
                if lower_arms:
                    self.setRPosture_srv("stand")
                if approved == "approved":
                    return True
                else:
                    return False
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("navigation as false")
            return False

    def start_moving(self, x: float,y: float,rotate: float) -> None:
        """
        Input: 
        x: The speed with which the robot will move forward or backward -> [-0.5,0.5]
        y: The speed with which the robot will move left or right -> [-0.5,0.5]
        rotate: The speed with which the robot will rotate left or right -> [-0.5,0.5]
        Output: None
        ----------
        The robots starts moving with the parameters given AND DOESN'T STOP until stopped (by calling stop_moving).
        """
        #print(f"going to move at {x} {y} {rotate}")
        movement_msg = Twist()
        movement_msg.linear.x = x
        movement_msg.linear.y = y
        movement_msg.angular.z = rotate
        self.move_publisher.publish(movement_msg)


    def stop_moving(self) -> None:
        """
        Input: None
        Output: None
        ----------
        The robots stops moving.
        """
        
        movement_msg = Twist()
        movement_msg.linear.x = 0
        movement_msg.linear.y = 0
        movement_msg.angular.z = 0
        self.move_publisher.publish(movement_msg)
        self.linear_vel = 0


    def start_yolo_awareness(self,command: bool, label="person") -> bool:
        """
        Input: 
        command: Whether to start following 
        speed: The speed with which the robot will move forward or backward -> [0.0,0.5]
        Output: True if the service was called correctly, False if not
        ----------
        Starts following the closest person to the robot
        """
        if self.pytoolkit:
            try:
                if command:
                    self.yolo_awareness_active = command
                    yolo_awareness_thread = Thread(target=self.yolo_awareness_srv_thread, args=([label]))
                    yolo_awareness_thread.start()
                    label_thread = Thread(target=self.get_closer_label, args=([label]))
                    label_thread.start()
                    print("Started yolo awareness")
                else:
                    self.yolo_awareness_active = command
                    print("Finished yolo awareness")
                    return True
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("pytoolkit as false")
            return False

    def follow_you(self, speed=None, awareness=True, avoid_obstacles=False, rotate=False) -> bool:
        """
        Input: 
        command: Whether to start following whoever is closests or not.
        speed: The speed with which the robot will move forward or backward -> [0.0,0.5]
        avoid_obstacles: If the robot should try to avoid obstacles or not.
        awareness: If the robot should use its head to follow the target or not.
        rotate: If the robot should ask the person to go in front of the robot to follow them or not.
        Output: True if the service was called correctly, False if not
        ----------
        Starts following the closest person to the robot
        """
        self.subscriber_yolo = rospy.Subscriber("/pytoolkit/ALMotion/failed",speech_recognition_status_msg,self.callback_move_failed)
        if self.pytoolkit and self.navigation and self.perception and self.speech:
            try:
                self.start_yolo_awareness(awareness)
                self.follow_you_active = True
                self.head_thread = True
                self.set_move_arms_enabled(False)
                if rotate:
                    self.start_yolo_awareness(False)
                    self.setRPosture_srv("stand")
                    self.talk("I will start rotating, please stand in the direction you want me to follow you and wait until i look at you and touch my head!",language="English",wait=False)
                    self.constant_spin_srv(15)
                    self.head_touched = False
                    self.waiting_head_touch = True
                    last_talk_time = rospy.get_time()
                    start_time = rospy.get_time()
                    while (not self.head_touched) and rospy.get_time() - start_time < 15:
                        if rospy.get_time()-last_talk_time > 5:
                            self.talk("Touch my head to get going!","English",wait=False)
                            last_talk_time = rospy.get_time()
                    self.robot_stop_srv()
                    self.waiting_head_touch = False
                    self.start_yolo_awareness(True)
                follow_thread = Thread(target=self.follow_you_srv_thread,args=([speed,awareness,avoid_obstacles]))
                follow_thread.start()
                person_thread = Thread(target=self.get_closer_person)
                person_thread.start()
                self.talk("I will follow you now! Please touch my head when we arrive. If i can't see you anymore please come back!", "English",wait=False)
                self.head_touched = False
                self.waiting_head_touch = True
                last_talk_time = rospy.get_time()
                start_time = rospy.get_time()
                print("Started following")
                while (not self.head_touched):
                    if rospy.get_time()-last_talk_time > 15:
                        self.talk("Remember to touch my head when we arrive","English",wait=True)
                        last_talk_time = rospy.get_time()
                self.waiting_head_touch = False
                self.start_yolo_awareness(False)
                self.set_move_arms_enabled(True)
                self.follow_you_active = False
                self.head_thread = False
                self.stop_moving()
                print("Finished following")
                return True
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            for tool in [self.pytoolkit,self.navigation,self.perception,self.speech]:
                if not tool:
                    print(f"{tool} as false")
            return False
        
    def calibrate_follow_distance(self,max_width,min_width):
        """
        Input:
        max_width: Maximum distance at which a person is considered too close
        min_width: Minimum distance at which a person is considered too far
        Output: None
        ----------
        Helps the followed person stay within a good distance of the robot
        """
        centered = False
        start_time = time.time()
        end_time = time.time()
        #0.2 is 20% of the total range
        close_threshold = max_width-(max_width - min_width)*0.10 #116
        far_threshold= min_width+(max_width-min_width)*0.10 #74
        while (not centered) and self.follow_you_active:
            followed_person = self.closest_person
            screen_center = 360 / 2
            person_x = followed_person[1]
            distance_from_center = person_x - screen_center
            person_width = followed_person[3]
            if "person" in self.labels: 
                self.iterations=0
                end_time = time.time()
                if end_time-start_time >= 2:
                    if(person_width>=close_threshold):
                        self.talk("You're too close", "English", True, False)
                    if(person_width<=far_threshold and distance_from_center<30):
                        self.talk("You're too far", "English", True, False)
                    start_time = time.time()
                if (int(person_width) in range(int(far_threshold),int(close_threshold))) or distance_from_center>15:
                    centered = True
                    continue
            else:
                self.iterations+=1
            if self.iterations >= 15:
                self.talk("I have lost you! Please come back", "English", True, False)
                while not ("person" in self.labels):
                    rospy.sleep(0.03)
        self.talk("Perfect. Try to keep at that distance", "English", True, False)
        self.iterations=0

    def go_back(self) -> None:
        """
        Input: None
        Output: None
        ----------
        Service to make the robot return to its last place.
        """
        self.go_to_place(place_name = self.last_place)

    def center_head_with_label(self, label_info,height=-0.3,resolution=1, desfase = 0) -> None:   
        """
        Input:
        label_info: tuple that represents the label to center
        Output: None
        ----------
        Service to center a label.
        """
        toggle_msg =  set_angle_srvRequest()
        toggle_msg.name = ["HeadYaw"]
        toggle_msg.angle = []
        toggle_msg.speed = 0
        self.toggle_get_angles_topic_srv(toggle_msg)
        label_width = label_info[3]
        label_center = (label_info[1] + label_width/2)
        # 54 grados caben en la camara y hay 320 pixeles en resolution 1
        factor = 54 / 320
        if resolution == 2:
            # 54 grados caben en la camara y hay 640 pixeles en resolution 2
            factor = 54 / 640
        label_degree_yolo = (label_center*factor) - 27 - desfase
        rospy.sleep(1)
        current_head_angle = self.angles
        label_degree = math.radians(label_degree_yolo - current_head_angle)
        self.set_angles_srv(["HeadYaw","HeadPitch"],[-label_degree, height],0.1)
        toggle_msg =  set_angle_srvRequest()
        toggle_msg.name = ["None"]
        toggle_msg.angle = []
        toggle_msg.speed = 0
        self.toggle_get_angles_topic_srv(toggle_msg)
    
    def yolo_awareness_srv_thread(self, label: str, speed=0.1) -> None:   
        """
        Input:
        speed: The speed with which the robot will move forward or backward -> [0.0,0.5]
        Output: None
        ----------
        A thread dedicated to tracking a label.
        """
        toggle_msg =  set_angle_srvRequest()
        toggle_msg.name = ["HeadYaw"]
        toggle_msg.angle = []
        toggle_msg.speed = 0
        self.toggle_get_angles_topic_srv(toggle_msg)
        while self.yolo_awareness_active: 
            rospy.sleep(0.02)
            # If a person was found
            if label in self.labels:
                # If that person is the closest person
                followed_label = self.closest_label
                label_width = followed_label[3]
                label_center = (followed_label[1] + label_width/2)
                label_degree_yolo = (label_center*0.16875) - 27
                current_head_angle = self.angles
                label_degree = math.radians(label_degree_yolo - current_head_angle)
                self.set_angles_srv(["HeadYaw","HeadPitch"],[-label_degree, 0],speed)
                
            
    def follow_you_srv_thread(self, speed=None, awareness=True, avoid_obstacles=False) -> None:
        """
        Input:
        speed: The speed with which the robot will move forward or backward -> [0.0,0.5]
        Output: None
        ----------
        A thread dedicated to following the person.
        """
        # Number of iterations with no person in sight
        self.iterations=0
        close = False
        backing_up = False
        navigation_attempts = 0
        iterations_no_safety_stop = 0
        calculated_vel = self.linear_vel
        # Variables to check if too close or too far
        # 140 is a value that worked well in testing
        max_width = 140
        # 55 is a value that worked well in testing
        min_width = 30
        speed = 0.5/(max_width-min_width)if speed is None else speed
        angular_vel = 0
        if awareness:
            joints_msg =  set_angle_srvRequest()
            joints_msg.name = ["HeadYaw"]
            joints_msg.angle = []
            joints_msg.speed = 0
            self.toggle_get_angles_topic_srv(joints_msg)
        while self.follow_you_active: 
            # If a person was found
            if "person" in self.labels:
                # If that person is the closest person
                followed_person = self.closest_person
                person_width = followed_person[3]
                # Person seen
                self.iterations=0
                # Calculate moving speed
                # a = (0.1-0.5)/(max_width-min_width)**2
                calculated_vel = speed*(max_width-person_width)+0.05 if person_width <max_width else 0.01
                # calculated_vel = (a*(person_width-min_width)**2)+0.5 if person_width <max_width else 0
                person_center = (followed_person[1] + person_width/2)
                person_degree_yolo = (person_center*0.16875) - 27
                current_head_angle = self.angles
                person_degree = person_degree_yolo - current_head_angle
                # 0.0025 worked well
                angular_vel = -min(0.5,abs(person_degree*(0.5/45)))*(person_degree/abs(person_degree))
                if person_width>=max_width:
                    close = True
                    if self.linear_vel != 0:
                        self.stop_moving()
                else:
                    close = False

                if person_width<min_width:
                    if self.linear_vel != 0:
                        self.stop_moving()
                        self.talk("You're too far", "English", wait=True, animated=False)
                        self.calibrate_follow_distance(max_width,min_width)
                        continue
                
                # If the rotation is big or the robot is not moving
                change_motion =(abs(angular_vel) > 0.07 and person_width>min_width) or abs(self.linear_vel-calculated_vel)>0.025
                if change_motion:
                    self.linear_vel=calculated_vel
                    if not close:
                        # Start moving and rotating to center person in frame
                        # Slower linear speed for turns
                        self.start_moving(self.linear_vel/3, 0, angular_vel)
                        rospy.sleep(0.2)
                        # Stop rotating but keeping moving forward
                        self.start_moving(self.linear_vel, 0, 0)
                    else:
                        self.start_moving(0, 0, angular_vel)
                        rospy.sleep(0.2)
                        self.stop_moving()
                if self.stopped_for_safety:
                    self.stopped_for_safety = False
                    # Try to run over the obstacle again
                    self.stop_moving()
                    self.talk("Please wait!", wait=True, animated=False)
                    self.start_moving(0, 0, angular_vel)
                    rospy.sleep(0.5)
                    self.start_moving(self.linear_vel/3, 0, angular_vel)
                    rospy.sleep(0.2)
                    # Stop rotating but keeping moving forward
                    self.start_moving(self.linear_vel, 0, 0)
                if avoid_obstacles:
                    if self.stopped_for_safety and not backing_up:
                        if navigation_attempts<2:
                            navigation_attempts+=1
                            iterations_no_safety_stop = 0
                            backing_up = True
                            self.stop_moving()
                            rospy.sleep(1.5)
                            self.start_moving(-0.2, 0, 0)
                            rospy.sleep(1)
                            self.stop_moving()
                            backing_up = False
                            self.stopped_for_safety = False
                            # Try to run over the obstacle again
                            self.start_moving(self.linear_vel/3, 0, angular_vel)
                            rospy.sleep(0.2)
                            # Stop rotating but keeping moving forward
                            self.start_moving(self.linear_vel, 0, 0)
                        else:
                            backing_up = True
                            self.stop_moving()
                            rospy.sleep(1.5)
                            self.start_moving(-0.2, 0, 0)
                            rospy.sleep(2)
                            self.stop_moving()
                            backing_up = False
                            navigation_attempts=0
                            self.avoiding_obstacle = True
                            self.talk("I found an obstacle in my path, i will attempt to go around it, if i fail, please help me remove it", wait=False, animated=False)
                            current_position = self.get_absolute_position_proxy()
                            current_x = current_position.x
                            current_y = current_position.y
                            current_angle = current_position.theta
                            theta_radians = math.radians(current_angle)

                            # Calcular las nuevas coordenadas
                            x_new = current_x + 2 * math.cos(theta_radians)
                            y_new = current_y + 2 * math.sin(theta_radians)
                            
                            self.add_place("start_avoid")
                            self.add_place("end_avoid",with_coordinates=True,x=x_new,y=y_new,theta=current_angle, edges=["start_avoid"])
                            self.set_current_place("start_avoid")
                            self.go_to_place("end_avoid",lower_arms=False)
                            self.avoiding_obstacle = False
                            self.talk("I'm done avoiding the obstacle, i will continue following you", wait=False, animated=False) 
                    else:
                        iterations_no_safety_stop += 1
                        if iterations_no_safety_stop >7:
                            navigation_attempts=0
            else:
                self.iterations+=1

            # If a person has not been seen for 30 iterations
            if self.iterations >=20:
                self.stop_moving()
                self.calibrate_follow_distance(max_width,min_width)
                continue
            time.sleep(0.02)


    def robot_stop_srv(self) -> bool:
        """
        Input: None
        Output: True if the service was called correctly, False if not
        ----------
        Stops the robot
        """
        if self.navigation:
            try:
                self.set_move_arms_enabled(False)
                approved = self.robot_stop_proxy()
                if approved == "approved":
                    return True
                else:
                    return False
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("navigation as false")
            return False

    def spin_srv(self, degrees: float):
        """
        Input: degrees
        Output: True if the service was called correctly, False if not
        ----------
        Spins the robot a number of degrees
        """
        if self.navigation:
            try:
                self.set_move_arms_enabled(False)
                approved = self.spin_proxy(degrees)
                if approved == "approved":
                    return True
                else:
                    return False
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("navigation as false")
            return False

    def go_to_defined_angle_srv(self, degrees: float):
        """
        Input: degrees
        Output: True if the service was called correctly, False if not
        ----------
        Goes to defined angle
        """
        if self.navigation:
            try:
                self.set_move_arms_enabled(False)
                approved = self.go_to_defined_angle_proxy(degrees)
                if approved == "approved":
                    return True
                else:
                    return False
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("navigation as false")
            return False

    def get_route_guidance_srv(self, place_name: str):
        """
        Input: place_name
        Output: True if the service was called correctly, False if not
        ----------
        Gives instructions in steps to get to the <place_name>
        """
        if self.navigation:
            try:
                self.set_move_arms_enabled(False)
                instructions = self.get_route_guidance_proxy(place_name)
                return instructions
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("navigation as false")
            return False

    def constant_spin_srv(self, velocity: float) -> bool:
        """
        Input: None
        Output: True if the service was called correctly, False if not
        ----------
        Starts constant spin at a <velocity> in degrees/sec (5-25)
        """
        if self.navigation:
            try:
                self.set_move_arms_enabled(False)
                approved = self.constant_spin_proxy(velocity)
                if approved == "approved":
                    return True
                else:
                    return False
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("navigation as false")
            return False

    def wait_go_to_place(self) -> bool:
        """
        Input: None
        Output: True if the service was called correctly, False if not
        ----------
        Waits for the robot to reach the place when navigating
        """
        if self.navigation:
            try:
                self.set_move_arms_enabled(False)
                print("Waiting to reach the place")
                finish = False
                response = False
                self.simpleFeedbackSubscriber = rospy.Subscriber(
                    "/navigation_utilities/simple_feedback",
                    simple_feedback_msg,
                    self.callback_simple_feedback_subscriber,
                )
                while not finish:
                    rospy.sleep(0.05)
                    if self.navigation_status == 2:
                        finish = True
                        response = True
                    elif rospy.is_shutdown():
                        finish = True
                        response = False
                return response
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("perception as false")
            return False

    def add_place(self, name: str, persist=0, edges=[], with_coordinates=False,x=0,y=0,theta=0) -> bool:
        """
        Input: name, edges, persist
        Output: True if the service was called correctly, False if not
        ----------
        Adds a place to the graph
        """
        if self.navigation:
            try:
                self.set_move_arms_enabled(False)
                if with_coordinates:
                    approved = self.add_place_with_coordinates_proxy(name, persist, edges,x,y,theta)
                else:
                    approved = self.add_place_proxy(name, persist, edges)
                if approved == "approved":
                    return True
                else:
                    return False
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("navigation as false")
            return False
        
    def center_object(self, object_name: str, movement_mode: str):
        """
        Input:
        object_name: (str) The name of the object which the robot wants to be at the center of the screen
        movement_mode: (str) Movement mode to center the object. Angular for the robot to rotate or translational for the robot to move left or right.
        Output: None
        ----------
        Helps the followed person stay within a good distance of the robot
        """
        if self.pytoolkit:
            try:
                object_thread = Thread(target=self.get_closest_object,args=([object_name]))
                object_thread.start()
                self.center_active = True
                center_x = 180 # 360 degrees / 2
                while self.center_active: 
                    # If the object was found
                    if object_name in self.labels:
                        # If that object is the closest instance of that object type
                        followed_object = self.closest_object
                        target_x = followed_object[1]
                        error_x = target_x - center_x
                        if movement_mode ==  "translational":
                            # 0.005 worked well
                            linear_vel = 0.005 * error_x
                            # If the change in speed is big enough
                            if (abs(linear_vel) > 0.1):
                                # Start moving to center the object in frame
                                self.start_moving(linear_vel, 0, 0)
                                rospy.sleep(0.2)
                                self.stop_moving()
                            else:
                                # Stop centering since the rotation is not big enough
                                self.center_active = False
                        elif movement_mode == "angular":
                            # 0.005 worked well
                            angular_vel = 0.005 * error_x
                            # If the rotation is big enough
                            if (abs(angular_vel) > 0.1):
                                # Start rotating to center the object in frame
                                self.start_moving(0, 0, angular_vel)
                                rospy.sleep(0.2)
                                self.stop_moving()
                            else:
                                # Stop centering since the rotation is not big enough
                                self.center_active = False
                print(f"Started centering object: {object_name}")
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
        else:
            print("pytoolkit as false")


    def align_with_object(self, object_name: str) -> bool:
        if self.navigation and self.perception:
            object_found = False
            while not object_found:

                if object_name in self.labels:
                    object_data = self.labels[object_name][0]
                    centered_point = (315 / 2) - object_data[1]
                    while not (-5 <= centered_point <= 5):
                        print(centered_point)
                        if object_name in self.labels:
                            object_data = self.labels[object_name][0]
                            centered_point = (315 / 2) - object_data[1]

                            if centered_point < 0:
                                self.go_to_relative_point(0.0, 0.02, 0.0)
                            else:
                                self.go_to_relative_point(0.0, -0.02, 0.0)
                            time.sleep(0.5)
                        else:
                            print(f"I have lost the {object_name}")
                            self.go_to_relative_point(0.0, 0.01, 0.0)
                            time.sleep(0.2)
                            object_found = False
                    return True
                else:
                    print(f"I have lost the {object_name}")
                    self.go_to_relative_point(0.0, 0.01, 0.0)
                    time.sleep(0.2)
                    object_found = False
        else:
            print("navigation and perception as false")
            return False

    ############ MANIPULATION SERVICES ###############

    def go_to_pose(self, pose: str, velocity=0.05) -> bool:
        """
        Input: pose options ->("bowl","box","cylinder","medium_object", "small_object_left_hand","small_object_right_hand","tray","head_up","head_down","head_default")
        Output: True if the service was called correctly, False if not
        ----------
        Goes to pose with hands or head
        """
        if self.manipulation:
            try:
                approved = self.go_to_pose_proxy(pose, velocity)
                if approved == "OK":
                    return True
                else:
                    return False
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("manipulation as false")
            return False
        
    def motion_tools_service(self):
        """
        Enables the motion Tools service from the toolkit of the robot.
        """
        request = motion_tools_msg()
        request.command = "enable_all"
        print("Waiting for motion tools service")
        rospy.wait_for_service('/robot_toolkit/motion_tools_srv')
        try:
            motion = rospy.ServiceProxy('/robot_toolkit/motion_tools_srv', motion_tools_srv)
            motion(request)
            print("Motion tools service connected!")
        except rospy.ServiceException as e:
            print("Service call failed")
            
    
    def enable_breathing_service(self):
        """
        Enables the breathing animations of the robot.
        """
        request = set_open_close_hand_srvRequest()
        request.hand = "All"
        request.state = "True"
        print("Waiting for breathing service")
        rospy.wait_for_service("/pytoolkit/ALMotion/toggle_breathing_srv")
        try:
            toggle_breathing_proxy = rospy.ServiceProxy("/pytoolkit/ALMotion/toggle_breathing_srv", set_open_close_hand_srv)
            toggle_breathing_proxy(request)
            print("Breathing service connected!")
        except rospy.ServiceException as e:
            print("Service call failed")
        request = set_open_close_hand_srvRequest()
        request.hand = "Head"
        request.state = "False"
        print("Waiting for breathing service")
        rospy.wait_for_service("/pytoolkit/ALMotion/toggle_breathing_srv")
        try:
            toggle_breathing_proxy = rospy.ServiceProxy("/pytoolkit/ALMotion/toggle_breathing_srv", set_open_close_hand_srv)
            toggle_breathing_proxy(request)
            print("Breathing service connected!")
        except rospy.ServiceException as e:
            print("Service call failed")
    
    def head_srv_thread(self, head_position="default") -> None:
        """
        Input: 
        head_position: the different head positions the robot can take. Options are : "default", "up", "down"
        Output: None
        ---------
        Thread dedicated to keeping the robot's head up.
        (Pepper's head falls down when moving due to vibration)
        """
        while self.head_thread: 
            self.setMoveHead_srv.call(head_position)
            rospy.sleep(8)
    
    def posture_srv_thread(self, posture="stand") -> None:
        """
        Input: 
        posture: the different head postures the robot can take. Options are : "stand","rest"
        Output: None
        ---------
        Thread dedicated to keeping the robot's posture
        """
        while self.posture_thread: 
            self.setRPosture_srv("stand")
            rospy.sleep(8)
             

    def saveState(self, name: str) -> bool:
        """
        Input: state as name
        Output: True if the state was created or False if is not
        ---------
        Saves the current state of the robot's joints to a CSV file
        """
        if self.manipulation:
            try:
                name_msg = String()
                name_msg.data = name
                save_state = self.saveState_proxy(name_msg)
                if save_state:
                    return True
                else:
                    return False
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("manipulation as false")
            return False

    def ask_for_object(self, object_name: str) -> bool:
        """
        Input: object_name
        Output: True if the service was called correctly, False if not
        ----------
        Asks for someone to give the robot the <object_name>.
        """
        if self.manipulation and self.speech:
            try:
                self.talk("Could you place the "+object_name+" in my hands, please?","English",wait=False)
                self.go_to_pose("generic_grasp")
                self.go_to_pose("open_both_hands")
                self.set_move_arms_enabled(False)
                # Sleep for the robot not to leave before taking the item
                rospy.sleep(5)
                self.talk("Thank you!","English",wait=False)  
                return True
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("manipulation and speech as false")
            return False

    def give_object(self, object_name: str) -> bool:
        """
        Input: object_name
        Output: True if the service was called correctly, False if not
        ----------
        Asks for someone to take the <object_name> from the robot.
        """
        if self.manipulation and self.speech:
            try:
                self.talk("Please pick up the "+object_name,"English",wait=False)
                rospy.sleep(5)
                self.set_move_arms_enabled(True)
                self.setRPosture_srv("stand") 
                self.talk("Thank you!","English",wait=False)     
                return True
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("manipulation and speech as false")
            return False

    ################ PYTOOLKIT ################

    def show_topic(self, topic: str) -> bool:
        """
        Input: topic
        Output: True if the service was called correctly, False if not
        ----------
        Displays the topic on the screen of the robot
        """
        if self.pytoolkit:
            try:
                approved = self.show_topic_proxy(topic)
                if approved == "OK":
                    return True
                else:
                    return False
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("pytoolkit as false")
            return False

    def show_image(self, image_path: str) -> bool:
        """
        Input: image_path or allreade saved image options -> ("sinfonia")
        Output: True if the service was called correctly, False if not
        ----------
        Displays the image on the screen of the robot
        """
        if self.pytoolkit:
            try:
                request = tablet_service_srvRequest()
                request.url = image_path
                approved = self.show_image_proxy(request)
                if approved == "OK":
                    return True
                else:
                    return False
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("pytoolkit as false")
            return False

    def show_video(self, video_url: str) -> bool:
        """
        Input: video_url url to a video in .mp4 format options -> ("sinfonia")
        Output: True if the service was called correctly, False if not
        ----------
        Displays the video on the screen of the robot
        """
        if self.pytoolkit:
            try:
                request = tablet_service_srvRequest()
                request.url = video_url
                approved = self.show_video_proxy(request)
                if approved == "OK":
                    return True
                else:
                    return False
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("pytoolkit as false")
            return False

    def hide_tablet(self) -> bool:
        """
        Output: True if the service was called correctly, False if not
        ----------
        Hdes whatever is on the tablet
        """
        if self.pytoolkit:
            try:
                approved = self.hide_proxy()
                if approved == "OK":
                    return True
                else:
                    return False
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("pytoolkit as false")
            return False

    def show_web_view(self, web_url: str) -> bool:
        """
        Input: web_url for a website
        Output: True if the service was called correctly, False if not
        ----------
        Displays a website on the screen of the robot
        """
        if self.pytoolkit:
            try:
                request = tablet_service_srvRequest()
                request.url = web_url
                approved = self.show_web_proxy(request)
                if approved == "OK":
                    return True
                else:
                    return False
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("pytoolkit as false")
            return False

    def set_awareness(self, state: bool) -> bool:
        """
        Input: True turn on || False turn off
        Output: True if the service was called correctly, False if not
        ----------
        Sets the awareness of the robot
        """
        if self.pytoolkit:
            try:
                approved = self.awareness_proxy(state)
                if approved == "OK":
                    return True
                else:
                    return False
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("pytoolkit as false")
            return False
        
    def play_animation(self, animation_name):
        anim_msg = animation_msg()
        anim_msg.family = "animations"
        anim_msg.animation_name = animation_name
        self.animationPublisher.publish(anim_msg)
        rospy.Timer(rospy.Duration(5), lambda event: self.setLedsColor(255, 255, 255), oneshot=True)

    def set_angles_srv(self, joints: list, angles:list, speed:float) -> bool:
        """
        joints: The list of joints to move to the desired angle.
        angles: The joints move the angles toward this angle.
        speed: Speed at which the angles are moved.
        ----------
        Moves the robots joints toward an angle at a set speed
        """
        if self.pytoolkit:
            try:
                joints_msg =  set_angle_srvRequest()
                joints_msg.name = joints
                joints_msg.angle = angles
                joints_msg.speed = speed
                approved =self.set_angles_proxy(joints_msg)
                if approved == "OK":
                    return True
                else:
                    return False
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("pytoolkit as false")
            return False

    def set_autonomous_life(self, state: bool) -> bool:
        """
        Input: True turn on || False turn off
        Output: True if the service was called correctly, False if not
        ----------  
        Sets the autonomous life of the robot
        """
        if self.pytoolkit:
            try:
                approved = self.autonomous_life_proxy(state)
                if approved == "OK":
                    return True
                else:
                    return False
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("pytoolkit as false")
            return False
        
    def set_security_distance(self, state:bool) -> bool: 
        if self.pytoolkit: 
            try: 
                approved = self.set_security_distance_proxy(state)
                if approved == "OK": 
                    return True
                else: 
                    return False
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("pytoolkit as false")
            return False
    
    def set_move_arms_enabled(self, state:bool)->bool: 
        if self.pytoolkit: 
            try: 
                approved = self.setMoveArms_srv(state,state)
                if approved == "OK": 
                    return True
                else: 
                    return False
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("pytoolkit as false")
            return False
        
    def setLedsColor(self,r,g,b):
        """
        Function for setting the colors of the eyes of the robot.
        Args:
        r,g,b numbers
            r for red
            g for green
            b for blue
        """
        ledsMessage = leds_parameters_msg()
        ledsMessage.name = "FaceLeds"
        ledsMessage.red = r
        ledsMessage.green = g
        ledsMessage.blue = b
        ledsMessage.time = 0
        self.ledsPublisher.publish(ledsMessage)
        rospy.sleep(0.2)

    def enable_breathing_service(self, chain, state):
        """
        Enables the breathing animations of the robot.
        """
        request = set_open_close_hand_srvRequest()
        request.hand = chain  # can be All, Body, Legs, Arms, LArm, RArm, Head
        request.state = str(state)  # can be True or False
        self.toggle_breathing_proxy(request)


    def gen_anim_msg(self, animation):
        """
        Generates an animation message to publish in the /animations topic for the robot to play that behaviour
        """
        
        anim_msg = animation_msg()
        anim_msg.family = "animations"
        anim_msg.animation_name = animation
        return anim_msg
    
    ################ SUBSCRIBER CALLBACKS ################

    def callback_look_for_object(self, data):
        self.object_found = data.data
        return data

    def callback_simple_feedback_subscriber(self, msg):
        self.navigation_status = msg.navigation_status

    def callback_sensor_subscriber(self, msg: touch_msg):
        if "head" in msg.name:
            if self.waiting_head_touch:
                if msg.state:
                    self.head_touched = msg.state
            else:
                self.head_touched = msg.state
        elif "hand_left_back" in msg.name:
            if self.waiting_left_touch:
                if msg.state:
                    self.hand_touched = msg.state
                    self.left_hand_touched = msg.state
            else:
                self.hand_touched = msg.state
                self.left_hand_touched = msg.state
        elif "hand_right_back" in msg.name:  
            if self.waiting_right_touch:
                if msg.state:
                    self.hand_touched = msg.state
                    self.right_hand_touched = msg.state
            else:
                self.hand_touched = msg.state
                self.right_hand_touched = msg.state
    

    def callback_move_failed(self, msg):
        if not self.avoiding_obstacle:
            if "Safety" in msg.status:
                self.stopped_for_safety = True

    def callback_get_angles(self, msg):
        self.angles = math.degrees(msg.angles[0]) 
            
    def callback_get_labels_subscriber(self, msg):
        self.labels = {}
        labels_msg = msg.labels
        x_coordinates_msg = msg.x_coordinates
        y_coordinates_msg = msg.y_coordinates
        widths = msg.widths
        heights = msg.heights
        ids = msg.ids
        for label in range(len(labels_msg)):
            if labels_msg[label] not in self.labels:
                self.labels[labels_msg[label]] = [(ids[label], x_coordinates_msg[label], y_coordinates_msg[label], widths[label], heights[label])]
            else:
                self.labels[labels_msg[label]].append((ids[label], x_coordinates_msg[label], y_coordinates_msg[label], widths[label], heights[label]))
                
    def callback_get_clothes_color_subscriber(self, msg):
        self.clothes_color = msg.colors[0]