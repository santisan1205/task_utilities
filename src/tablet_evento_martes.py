#!/usr/bin/env python3
import rospy
from fastapi import FastAPI, Request
from fastapi.staticfiles import StaticFiles
from fastapi.responses import RedirectResponse, HTMLResponse
from fastapi.middleware.cors import CORSMiddleware
import os
import uvicorn
import threading
from robot_toolkit_msgs.msg import speech_msg, text_to_speech_status_msg, audio_tools_msg, animation_msg, leds_parameters_msg, motion_tools_msg
from robot_toolkit_msgs.srv import audio_tools_srv, set_open_close_hand_srv, set_open_close_hand_srvRequest, motion_tools_srv, battery_service_srv, tablet_service_srv

# Initialize ROS node
rospy.init_node("tablet_evento_node")

# --- ROS Speech Integration ---
try:
    rospy.wait_for_service('/robot_toolkit/audio_tools_srv', timeout=5.0)
    audioToolsService = rospy.ServiceProxy('/robot_toolkit/audio_tools_srv', audio_tools_srv)
    enableSpeech = audio_tools_msg()
    enableSpeech.command = "enable_tts"
    audioToolsService(enableSpeech)
    rospy.loginfo("TTS enabled on robot.")
except (rospy.ServiceException, rospy.ROSException) as e:
    rospy.logerr(f"Failed to connect to audio_tools_srv: {e}")

def motion_tools_service():
    """
    Enable motion tools on the robot by calling /robot_toolkit/motion_tools_srv.
    """
    try:
        rospy.wait_for_service('/robot_toolkit/motion_tools_srv', timeout=5.0)
        motion_proxy = rospy.ServiceProxy('/robot_toolkit/motion_tools_srv', motion_tools_srv)
        req = motion_tools_msg()
        req.command = "enable_all"
        motion_proxy(req)
        rospy.loginfo("Motion tools enabled on robot.")
    except (rospy.ServiceException, rospy.ROSException) as e:
        rospy.logerr(f"Failed to connect to motion_tools_srv: {e}")

# Try to initialize motion tools early
try:
    motion_tools_service()
except Exception as e:
    rospy.logerr(f"Could not initialize motion tools: {e}")

def enable_breathing(chain, enable):
    """
    Enables or disables the breathing animation for a specific body part.
    """
    rospy.wait_for_service("/pytoolkit/ALMotion/toggle_breathing_srv", timeout=2.0)
    try:
        toggle_breathing_proxy = rospy.ServiceProxy("/pytoolkit/ALMotion/toggle_breathing_srv", set_open_close_hand_srv)
        request = set_open_close_hand_srvRequest()
        request.hand = chain
        request.state = str(enable)
        toggle_breathing_proxy(request)
        rospy.loginfo(f"Breathing for '{chain}' set to {enable}.")
    except (rospy.ServiceException, rospy.ROSException) as e:
        rospy.logerr(f"Breathing service call failed for chain '{chain}': {e}")

# Enable robot breathing on startup
try:
    enable_breathing("All", True)
    enable_breathing("Head", False)
except Exception as e:
    rospy.logerr(f"Could not enable breathing: {e}")

# Publishers used by the module
speech_pub = rospy.Publisher('/speech', speech_msg, queue_size=10)
animation_pub = rospy.Publisher('/animations', animation_msg, queue_size=10)
leds_pub = rospy.Publisher('/leds', leds_parameters_msg, queue_size=10)


show_picture_proxy = rospy.ServiceProxy('pytoolkit/ALTabletService/show_picture_srv', battery_service_srv)
show_web_view_proxy = rospy.ServiceProxy('pytoolkit/ALTabletService/show_web_view_srv', tablet_service_srv)
hide_proxy = rospy.ServiceProxy("/pytoolkit/ALTabletService/hide_srv",battery_service_srv)
show_picture_proxy()
show_web_view_proxy("http://157.253.113.200:8000/index.html")

robot_speaking = False
robot_name = rospy.get_param("~robot_name", "pepper")

def talk(key, language, animated=True, talk_speed=100):
    """
    Publishes a message to the /speech topic to make the robot talk.
    """
    global robot_speaking
    text_to_say = f"\\rspd={talk_speed}\\{key}"
    t2s_msg = speech_msg()
    t2s_msg.animated = animated
    t2s_msg.language = language
    t2s_msg.text = text_to_say
    speech_pub.publish(t2s_msg)
    print("Talking...")
    
    print(f"{robot_name.upper()} said: {key}")

def play_animation(animation_name):
    """
    Publish an animation request to /animations and reset LEDs to white after 5s.
    """
    try:
        anim_msg = animation_msg()
        anim_msg.family = "animations"
        anim_msg.animation_name = animation_name
        animation_pub.publish(anim_msg)
        # schedule leds reset to white (oneshot)
        def _reset_leds(event):
            leds_msg = leds_parameters_msg()
            leds_msg.name = "FaceLeds"
            leds_msg.red = 255
            leds_msg.green = 255
            leds_msg.blue = 255
            leds_msg.time = 0
            leds_pub.publish(leds_msg)
        rospy.Timer(rospy.Duration(5), _reset_leds, oneshot=True)
        rospy.loginfo(f"Played animation: {animation_name}")
    except Exception as e:
        rospy.logerr(f"Failed to play animation '{animation_name}': {e}")

app = FastAPI()

# Configure CORS
_allowed_origins = os.getenv("CORS_ORIGINS", "*")
if _allowed_origins.strip() == "*":
    _origins = ["*"]
else:
    _origins = [o.strip() for o in _allowed_origins.split(",") if o.strip()]

_allow_credentials = False if _origins == ["*"] else True

app.add_middleware(
    CORSMiddleware,
    allow_origins=_origins,
    allow_credentials=_allow_credentials,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Mount static files directory
static_path = os.path.join(os.path.dirname(__file__), "static")
app.mount("/static", StaticFiles(directory=static_path), name="static")

@app.get("/", response_class=RedirectResponse)
async def main_page():
    """
    Redirects the root URL to the main HTML page.
    """
    return RedirectResponse(url="/static/evento_martes.html")

# Lock para asegurar solo una acción en curso
action_lock = threading.Lock()

@app.post("/action", response_class=HTMLResponse)
@app.head("/action", response_class=HTMLResponse)
async def handle_action(action: str):
    """
    Handles actions from the web page and ensures only one action runs at a time.
    """
    if not action:
        return HTMLResponse("No action provided", status_code=400)

    # Intentar adquirir el lock sin bloquear. Si falla, devolver 409.
    if not action_lock.acquire(blocking=False):
        return HTMLResponse("Another action is running", status_code=409)

    try:
        if action == "greet":
            print("Action received: greet")
            talk("Hola! Soy Nova y vengo del universo Tirant Praim, para presentarte su nueva solución. Tirant praim Conversa. La IA jurídica más precisa y fiable. Descubra rápidamente de todo lo que es capaz de realizar", "Spanish")
        elif action == "dance":
            print("Action received: dance")
            play_animation("disco/full_launcher")
        elif action == "acabo_hablar":
            print("Action received: acabo_hablar")
            show_web_view_proxy("http://157.253.113.200:8000/menu.html")
            talk("Ahora que conoces brevemente de lo que es capaz de hacer. Es momento de que lo pruebes en directo. Acércate a los peces y realiza cualquier consulta jurídica y descubre todo el potencial de Tirant praim Conversa!", "Spanish")
            rospy.sleep(30)
            show_web_view_proxy("http://157.253.113.200:8000/index.html")
        elif action == "pose":
            talk("Tomemonos una foto, voy a posar", "Spanish", animated=False)
            enable_breathing("All", False)
            rospy.sleep(1)
            print("Action received: foto")
            play_animation("Gestures/ShowSky_8")
            rospy.sleep(10)
            enable_breathing("All", True)
        elif action == "foto":
            print("Action received: foto")
            talk("Te tomo una fotito!", "Spanish", animated=False)
            play_animation("Waiting/TakePicture_1")
            hide_proxy()
            rospy.sleep(2)
            show_picture_proxy()
            rospy.sleep(5)
            show_web_view_proxy("http://157.253.113.200:8000/menu.html")
        else:
            return HTMLResponse(f"Unknown action: {action}", status_code=400)

        return HTMLResponse("{}", status_code=200)
    finally:
        # Siempre liberar el lock
        action_lock.release()

if __name__ == "__main__":
    """
    Main entry point to run the FastAPI server.
    """
    uvicorn.run(app, host="0.0.0.0", port=8000)
