from code_generation.generate_utils import generate_response, load_task_config

# from generate_utils import generate_response, load_task_config
from code_generation.database.models import Model

class LongStringGenerator:

    def __init__(self) -> None:
        self.robot_vars = load_task_config()
        self.place_names = self.robot_vars["place_names"]
        self.objects = self.robot_vars["objects"]
        self.task_module_code = f"""
        Perception functions:
        self.tm.find_object(object_name)->bool: Returns True if the object was found, False if not, the only possible objects with their exact syntax are: {self.objects}.
        self.tm.count_objects(object_name)->int: Returns the number of objects found. The object can be anything, even objects not available in other services.
        self.tm.search_for_specific_person(class_type,specific_characteristic)->bool: The robot spins in place in the current room until it finds a person with a specific characteristic. The options for class_type are: "pointing", "name", "raised_hand". The options for specific_characteristic depend on class_type. If class_type is "name", options are human names like "Paris","Robin","Jaine". If class_type is "pointing" or "raised_hand" options are "right", "left", "center". Returns True if such a person was found, False if not
        self.tm.find_item_with_characteristic(class_type,characteristic,place)->str: Allows the robot to find the item which has the given "characteristic", the options for class_type are: "color", "size", "weight", "position", "description". the options for characteristic depend on class_type: Color ("red","blue","black with white dots"). Size ("smallest", "largest", "thinnest", "big one", "small one"). Weight ("lightest", "heaviest"). Position ("left", "right"). Description("fragile","container","pourable","two-handed"). "place" Indicates the furniture or item on top of which the robot needs to search for the object.Returns a String with the name of the object with the needed characteristics or "None" if it wasn't found
        self.tm.get_person_gesture()->str: Allows the robot to determine the gesture of the person the robot is looking. Returns a string with the gesture that the person is doing, if the robot couldn't tell the gesture, returns 'None'
        self.tm.get_all_items(place)->list: Returns a list of items on a piece of furniture.  "place" Indicates the furniture or item on top of which the items are. Place can be "" for all of the items visible
        
        Speech functions:
        self.tm.talk(text): Allows the robot to say the input of the service
        self.tm.speech2text_srv()->str: Allows the robot to listen to the user and returns the text that the robot heard
        self.tm.q_a(question)->str: Allows the robot to ASK a question and returns the answer of the user (filtering unnecesary content). The input of this function is the question the robot asks, and the return the answer of the person
        self.tm.answer_question(question)->bool: Returns a string with an answer to the question

        Navigation functions:
        self.tm.go_to_place(place_name): Allows the robot to calculate the route needed to go to a place, the only possible places with their exact syntax are: {self.place_names}. If the robot needs to go to an object, that is not a possible place, use the closest logical place
        self.tm.go_back(): Allows the robot to go to back to the starting place. Use this if a person in the starting place needs an answer that you got in another place.
        self.tm.follow_you(): Allows the robot to follow the user and stop when the head of the robot is touched. This service is a blocking call, meaning the program will not resume until the robot finishes following the person.
        self.tm.robot_stop_srv(): Makes the robot to stop

        Manipulation functions:
        self.tm.ask_for_object(object_name): Allows the robot to ask a person to place the object in its hands and then thank them. The object can be anything, even objects not available in other services.
        self.tm.give_object(object_name): Allows the robot to ask a person to receive the object in their hands and then thank them. The object can be anything, even objects not available in other services.
        """

    def generate_exec(self, task:str)-> str:
        print("generate exec")
        system_message = """You are a code generation AI model for a robot called Pepper. You only return code, nothing more."""

        text_prompt = f"""
        You are a Pepper robot, given a task definition and an interface of the codebase (it only describes what each function does). You must generate the python code that completes the task using the codebase interface.

        # Details about the code to generate:
        - Always try to complete the task
        - The task module has allready instantiated as `self.tm = task_module.Task_module(perception = True,speech=True,manipulation=True, navigation=True)` so you should never instantiate it again
        - Use the functions as they are described in the codebase interface, for example `self.tm.talk("Hello")` to talk
        - Do not use classes, just functions
        - If needed you can import libraries for extracting the actual time, date, etc.
        - The code must be written in python and the output will be executed directly
        - Always use self.tm.<function_name> to call the functions of the codebase interface
        - Return only the code, just code, your output is going to be saved in a variable and executed with exec(<your answer>)
        - Make sure to call and execute the functions from the codebase
        - If with the given functions and external libraries you cannot complete the task, please respond with self.tm.talk("I am sorry but I cannot complete this task")
        - MANDATORY: you must talk in between steps so users know what you are doing
        - The only available places are: {self.place_names}, if you need to go to a place that is not listed use the most similar one from the list. Not doing this will result in an error. Use the syntax from the list when calling the codebase functions.
        - If the place you need to go or a similar place is NOT listed above, please respond with self.tm.talk("I cannot go to <place>")
        - Always assume that the robot is at the entrance of the house at the beginning of the task
        - To locate objects, navigate to the place in which is most likely to find the object and then use the `find_object` service
        - If you need to go to a place that is not in: {self.place_names}, NEVER Invent it, instead use the closest logical place name in: {self.place_names}.
        - If a person in the starting place needs an answer from another room, the robot has to use the 'go_back' service, before saying the answer with the 'talk' service. For example, if the user says "Tell me the name of the person at the shelf", you need to go to the "kitchen", get the answer and use the 'go_back' service, before saying the answer. If you did not get the answer, allways go back before saying anything.
        - If you have to go to an object location that is not a place listed in the `go_to_place` service, use the most logical place name, and never use the object name UNLESS it's listed in the `go_to_place` service. For example if the user says "Guide Robin from the refrigerator to the living room", in the 'go_to_place' service call use "kitchen" instead of "refrigerator"
        - If you are using the `go_to_place` service to go to a place, do not make unnecesary stops or make intermediate pauses, the `go_to_place` service calculates the optimal route and should not be modified.
        - If you have to find a specific person, NEVER assume that the person is in the current place, always use the `go_to_place` service to go to the most appropiate place.
        - The robot is in a starting location
        - If the robot has to guide a person from one place to another, always the `go_to_place` service to go to the first place and then another `go_to_place` service to the second place
        - The user asking the request is NOT necessarily who the robot has to help, always search for them in the most logical place using the `go_to_place` service
        - The follow you service makes the robot follow the person in front of them, NOT the other way around. NEVER use this to guide someone.
        - If the robot is asked to find a person in a place and then find the same person in another place, ALWAYS go to both places and try to find them, even if the person was found in the first place. For example if the user says "Meet Charlie at the armchair then look for them in the bedroom", the robot should use the `go_to_place` service to meet charlie at the armchair, and then the `go_to_place` service again, to find them in the bedroom
        - If the robot is following a person, the robot should trust the person to take them to the appropiate place, and should NOT use the `go_to_place` service afterwards.
        - When asked to lead a person, the robot should NEVER use follow_you, instead the robot should use go_to_place.
        - If the robot is asked to find and grasp or take or bring an object, the robot should NEVER search for it with the 'find_object' service, instead just use the 'ask_for_object' service to ask for help to get the object, and then use the 'give_object' service to ask for help in delivering the object.
        - If the robot has to grasp or take or bring an object, that object can be anything, even objects not available: {self.objects}
        - If the robot has to count occurrences of an object, that object can be anything, even objects not available: {self.objects}
        - If the user wants the answer for himself for any given question, the robot MUST ALWAYS use the 'go_back' service before the 'talk' service, in order for the user to get the answer, otherwise the user will not get the desired answer
        - If the user asked the robot to tell them the answer DO NOT use the 'go_back' service after you've already said the answer, YOU MUST use 'go_back' before saying the answer to the main user
        - If the user wants the robot to answer something using the 'find_item_with_characteristic', the robot MUST ALWAYS use the 'go_back' service before the 'talk' service 
        - If the task is to take a person from a place to another, DO NOT use the 'ask_for_object' service, instead find the person in the desired room and ask them to follow the robot and then use the 'go_to_place' service to get there.
        - The Pepper Robot is a member of the SinfonIA Uniandes team for the Robocup 2024 competition. The team members are: Alonso Hernandez Tavera, Alexa Gutierrez Ballen, Santiago Rodriguez Mora, David Cuevas Alba, David Tobon Molina
        - If the `q_a` service is used to ask a question NEVER use the `talk` service to ask the question, the `q_a` service already makes the robot ask the question
        - If the task is to find a person or object, the robot must use the `talk` service to speak when that person or object is found.
        
        - The only available objects are: {self.objects}, if you need to recognize an object that is not listed use the most similar one from the list. Not doing this will result in an error. Use the syntax from the list when calling the codebase functions.
        - If the object you need to recognize or a similar object is NOT listed above, please respond with self.tm.talk("I cannot recognize <object>")
        - For recognizing people just use "person" instead of a specific name

        - If you need to ask a question that is not listed just use the `talk` method to say the question and the `speech2text_srv` followed to save the answer. Use the syntax from the list when calling the codebase functions.

        # Output Format:
        - Your output needs to be formatted in markdown as a python code snippet, do not add anything else to the output (don't add any exec calls or "Here's your code" statements), just the code.
        - DO NOT INCLUDE ANY AI CHAT RESPONSES IN THE OUTPUT, JUST GENERATE THE CODE
        - DO NOT INCLUDE ANY COMMENTS IN THE CODE
        - The response must be formatted as follows: ```python\n<CODE> ```

        For example, if the task is "Grab a bottle, and bring it to the living room" you should return:
        ```python
        self.tm.talk("I am going to grab a bottle")
        self.tm.go_to_place("kitchen")
        self.tm.ask_for_object("bottle")
        self.tm.talk("I am going to the living room")
        self.tm.go_to_place("living")
        self.tm.give_object("bottle")
        ```
        Without the asterisks

        # Task description:
        {task}

        # Codebase Interface (functions you can use):

        {self.task_module_code}

        # Code to generate:
        """
        return generate_response(text_prompt, system_message=system_message, is_code=True, model_type=self.model)

    def generate_code(self, task:str, model: Model)->str:
        self.model = model
        code = self.generate_exec(task)
        return code