#!/usr/bin/env python3
import uvicorn
import socket
import os
from fastapi import FastAPI, Request
from fastapi.responses import HTMLResponse
from fastapi.templating import Jinja2Templates
from fastapi.staticfiles import StaticFiles
from robot_fsm import MERCADITO

# --- Network Setup ---
def get_local_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(('10.255.255.255', 1))
        IP = s.getsockname()[0]
    except:
        IP = '127.0.0.1'
    finally:
        s.close()
    return IP

local_ip = get_local_ip()
base_url = f"http://{local_ip}:8001"
print(f"✅ Server URL: {base_url}")

app = FastAPI()
script_dir = os.path.dirname(os.path.abspath(__file__))
templates = Jinja2Templates(directory=os.path.join(script_dir, "templates"))
app.mount("/static", StaticFiles(directory=os.path.join(script_dir, "static")), name="static")

# --- Initialize Robot ---
robot = MERCADITO(base_url)

# --- Routes ---

@app.get("/", response_class=HTMLResponse)
async def index_page(request: Request):
    """ Served during INIT state """
    return templates.TemplateResponse("index.html", {"request": request})

@app.get("/menu", response_class=HTMLResponse)
async def menu_page(request: Request, text: str = "Listening..."):
    """ Served during HABLAR state """
    return templates.TemplateResponse("menu.html", {"request": request, "answer_text": text})

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8001)