import uvicorn
import socket
import os
from fastapi import FastAPI, Request
from fastapi.responses import HTMLResponse
from fastapi.templating import Jinja2Templates
from fastapi.staticfiles import StaticFiles
from robot_fsm import Preguntados
import threading
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
robot = Preguntados(base_url)
threading.Thread(target=robot.run, daemon=True).start()

# --- Web Page Endpoints ---
@app.get("/", response_class=HTMLResponse)
async def inicio(request: Request):
    return templates.TemplateResponse("inicio.html", {"request": request})

@app.get("/pregunta", response_class=HTMLResponse)
async def pregunta_page(request: Request):
    return templates.TemplateResponse("pregunta.html", {"request": request})

@app.get("/resultado", response_class=HTMLResponse)
async def resultado_page(request: Request):
    return templates.TemplateResponse("resultado.html", {"request": request})

# --- API Endpoints ---
@app.get("/api/pregunta")
async def api_pregunta():
    if not hasattr(robot, "current_answers"):
        return {"categoria": "", "pregunta": "", "opciones": []}

    return {
        "categoria": robot.current_category,
        "pregunta": robot.current_question,
        "opciones": robot.current_answers["options"]
    }

@app.get("/api/resultado")
async def api_resultado():
    return {
        "icono": getattr(robot, "ui_icon", "/static/trofeo.png"),
        "mensaje": getattr(robot, "ui_message", "Esperando partida..."),
        "puntaje": getattr(robot, "score", 0),
        "total": getattr(robot, "total_rounds", 0)
    }