#!/usr/bin/env python3
import uvicorn
import socket
import threading
import time
import os
from fastapi import FastAPI, Request
from fastapi.responses import HTMLResponse
from fastapi.templating import Jinja2Templates
from task_module import Task_module
from fastapi.staticfiles import StaticFiles

# --- 1. Dynamic IP Setup ---
def get_local_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(('10.255.255.255', 1))
        IP = s.getsockname()[0]
    except Exception:
        IP = '127.0.0.1'
    finally:
        s.close()
    return IP

local_ip = get_local_ip()
web_url = f"http://{local_ip}:8001/"
print(f"✅ Server URL: {web_url}")

# --- 2. Initialize Robot Wrapper ---
# Note: I kept your specific flags (speech=False, etc)
task = Task_module(speech=True, pytoolkit=True, manipulation=False)
robot_lock = threading.Lock()

# --- 3. FastAPI Setup ---
app = FastAPI()

script_dir = os.path.dirname(os.path.abspath(__file__))
templates_path = os.path.join(script_dir, "templates")

static_path = os.path.join(script_dir, "static") # <--- Define static path

# Tell FastAPI: "When someone asks for /static/filename, look in the static_path folder"
app.mount("/static", StaticFiles(directory=static_path), name="static")

templates = Jinja2Templates(directory=templates_path)

# Change @app.get to @app.api_route and add "HEAD" to the methods list
@app.api_route("/", methods=["GET", "HEAD"], response_class=HTMLResponse)
async def home(request: Request):
    return templates.TemplateResponse("index.html", {"request": request})

@app.post("/run/{action_name}")
async def run_action(action_name: str):
    if not robot_lock.acquire(blocking=False):
        return "⚠️ Robot is busy, please wait!"

    try:
        print(f"🤖 Executing: {action_name}")
        
        if action_name == "greet":
            # Using Spanish as requested in previous turn
            task.talk("Hola! Soy Nova y vengo del universo Tirant Praim.", "Spanish")
            
        elif action_name == "dance":
            task.play_animation("disco/full_launcher")
            
        elif action_name == "pose":
            task.talk("Tomemonos una foto.", "Spanish", animated=False)
            task.enable_breathing_service("All", False)
            task.play_animation("Gestures/ShowSky_8")
            task.enable_breathing_service("All", True)

        elif action_name == "acabo_hablar":
            task.show_web_view(web_url)
            task.talk("Acércate a los peces y realiza cualquier consulta.", "Spanish")

        else:
            return f"❓ Unknown Action: {action_name}"

        return f"✅ Done: {action_name}"

    except Exception as e:
        print(f"Error: {e}")
        return f"❌ Error: {e}"
    finally:
        robot_lock.release()

# --- 4. The Fix: Background Opener ---
def open_tablet_delayed():
    """Waits for the server to start, then opens the tablet."""
    print("⏳ Waiting 3 seconds for server to start...")
    time.sleep(3) 
    try:
        print(f"🚀 Sending URL to robot: {web_url}")
        task.show_web_view(web_url)
    except Exception as e:
        print(f"⚠️ Could not open tablet automatically: {e}")

if __name__ == "__main__":
    # Start the delay timer in a separate thread so it doesn't block uvicorn
    threading.Thread(target=open_tablet_delayed).start()
    
    # Start the server (Main thread stays here)
    uvicorn.run(app, host="0.0.0.0", port=8001)