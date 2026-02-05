
from openai import AzureOpenAI
import tiktoken
import re
import os
import code_generation.configs as cfg
from code_generation.database.models import Model
from requests.exceptions import JSONDecodeError
import requests

codebase = """"""

CONFIG_PATH = os.path.join("src/task_utilities/src/code_generation/configs/robot_vars.csv")
PlACES_PATH = os.path.join("src/navigation_utilities/resources/places.txt")
LABELS_PATH = os.path.join("src/perception_utilities/src/yolov7/coco.yaml")
TASK_VARS = None

def load_task_config()->dict:
    global TASK_VARS
    if TASK_VARS is None:
        config = {}
        with open(CONFIG_PATH,"r") as f:
            for line in f:
                splitted_line = line.split(":")
                key = splitted_line[0]
                value = splitted_line[1].strip().split(",")
                config[key] = value
        
        with open(PlACES_PATH,"r") as places_file:
            places = []
            for line in places_file:
                splitted_line = line.split(",")
                place = splitted_line[0].strip()
                last_field = splitted_line[4].strip()
                if last_field != "False" :
                    places.append(place)
            config["place_names"] = places
        
        with open(LABELS_PATH,"r") as labels_file:
            nombres = []
            for line in labels_file:
                linea = line.strip()
                if linea.startswith('- name:'):
                    nombre = linea.split(':', 1)[1].strip()
                    nombres.append(nombre)
            config["objects"] = nombres
        TASK_VARS = config
    return TASK_VARS

def load_code_gen_config():
    pass

def get_task_module_code()-> str:
    global codebase
    if codebase == "":
        with open(os.path.join(os.path.dirname(__file__), "task_module_interface.py"), "r") as f:
            codebase = f.read()
    return codebase

def count_tokens(string: str, encoding_name: str = "cl100k_base") -> int:
    """Returns the number of tokens in a text string."""
    encoding = tiktoken.get_encoding(encoding_name)
    num_tokens = len(encoding.encode(string))
    return num_tokens

def generate_response(text_prompt, system_message=None, is_code=True, model="gpt-3.5-turbo-16k", model_type= Model.GPT35, temperature=0):
    print("generate response")
    messages = [
        {"role": "user", "content": text_prompt}
    ]

    if system_message:
        messages = [
            {"role": "system", "content": system_message},
        ] + messages
    if model_type == Model.LLAMA2:
        response = requests.post("http://localhost:6969/llama2", json={"messages": messages})
        try:
            answer = response.json()[0]["generation"]["content"].rstrip()
        except JSONDecodeError:
            print("Error decoding json response from LLAMA2, please check that the server is running and that the model is loaded correctly")
            return None
    else:
        clientGPT = AzureOpenAI(
            azure_endpoint= "https://sinfonia.openai.azure.com/",
            api_key= os.getenv("GPT_API"),
            api_version="2024-02-01",
        )
        try:
            prediction = clientGPT.chat.completions.create(
                model="GPT-4o", 
                messages=messages, 
                temperature=temperature
            )
        except:
            return "self.tm.talk('I am sorry but I cannot complete this task')"
    if model_type != Model.LLAMA2:
        answer = prediction.choices[0].message.content
    if is_code:
        pattern = r'```python(.*?)```'
        try:
            return (re.search(pattern, answer, re.DOTALL).group(1)).strip()
        except AttributeError:
            pass
        try:
            pattern = r'```(.*?)```'
            return (re.search(pattern, answer, re.DOTALL).group(1)).strip()
        except AttributeError:
            return f"""#Error while getting response from model {model_type.value}. Response had an incorrect format.\n#The response was:\n{f'{answer}'}"""
    else:
        return answer