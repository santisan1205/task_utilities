#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
juego_memoria.py — Supervisor del Juego
Integración: FSM Original + Control Web
Descripción: La FSM controla el flujo general (Inicio -> Juego -> Fin), 
             pero la lógica detallada (cartas, puntajes) vive en la Tablet.
"""

import uvicorn
import socket
import os
import threading
import time
from fastapi import FastAPI, Request
from fastapi.responses import HTMLResponse
from fastapi.templating import Jinja2Templates
from fastapi.staticfiles import StaticFiles
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

# --- Imports Robot (Simulación si estás en PC) ---
try:
    import rospy
    from task_module import Task_module as TM
    from transitions import Machine
except ImportError:
    print("⚠️ MODO SIMULACIÓN: No se detectó ROS. Usando Mocks.")
    from unittest.mock import MagicMock
    rospy = MagicMock()
    TM = MagicMock()
    Machine = MagicMock()

# --- Configuración de Red ---
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
BASE_URL = f"http://{local_ip}:8001"
print(f"✅ URL DEL JUEGO: {BASE_URL}")

# --- Configuración FastAPI ---
app = FastAPI()
script_dir = os.path.dirname(os.path.abspath(__file__))
templates = Jinja2Templates(directory=os.path.join(script_dir, "templates"))
app.mount("/static", StaticFiles(directory=os.path.join(script_dir, "static")), name="static")

app.add_middleware(
    CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"],
)

# Modelos de datos para la API
class GameData(BaseModel):
    winner: str = None

# ==========================================
# TU MÁQUINA DE ESTADOS ORIGINAL (MODIFICADA)
# ==========================================

class JuegoMemoria(object):

    # TUS ESTADOS ORIGINALES (INTACTOS)
    STATES = [
        "INIT", "INTRO", "ASK_MODE", "SETUP_GAME",
        "LOOP", "SAY_RESULT", "ERROR_EXIT"
    ]

    # TUS TRANSICIONES ORIGINALES
    # Nota: Los triggers ahora serán llamados por la API, no por bucles internos.
    TRANSITIONS = [
        {"trigger": "start",        "source": "INIT",       "dest": "INTRO"},
        {"trigger": "go_ask_mode",  "source": "INTRO",      "dest": "ASK_MODE"},
        {"trigger": "mode_ok",      "source": "ASK_MODE",   "dest": "SETUP_GAME"},
        {"trigger": "setup_ok",     "source": "SETUP_GAME", "dest": "LOOP"},
        {"trigger": "say_results",  "source": "LOOP",       "dest": "SAY_RESULT"},
        {"trigger": "play_again",   "source": "SAY_RESULT", "dest": "ASK_MODE"},
        {"trigger": "error",        "source": "*",          "dest": "ERROR_EXIT"},
    ]

    def __init__(self):
        # Inicializar Pepper
        try:
            self.tm = TM(speech=True, pytoolkit=True)
            self.tm.initialize_node("JUEGO_MEMORIA_FSM")
            self.tm.initialize_pepper()
        except:
            pass

        self.tts_lang = "Spanish"
        self.winner_name = "" 

        # FSM
        self.machine = Machine(model=self, states=self.STATES, transitions=self.TRANSITIONS, initial="INIT")

    # ---------- Wrappers del Robot ----------
    def say(self, text: str):
        print(f"🤖 Pepper dice: {text}")
        try:
            self.tm.talk(text=text, language=self.tts_lang)
        except:
            pass

    def show_tablet(self):
        try:
            url = f"{BASE_URL}/"
            self.tm.show_web_view(url)
        except:
            pass

    # ---------- ESTADOS (La Nueva Lógica "Pasiva") ----------

    def on_enter_INIT(self):
        print("--- ESTADO: INIT ---")
        self.show_tablet()
        time.sleep(2)
        self.start() # Auto-transición a INTRO

    def on_enter_INTRO(self):
        print("--- ESTADO: INTRO ---")
        self.say("Hola. Vamos a jugar memoria. Mira mi pantalla.")
        self.go_ask_mode() # Auto-transición a ASK_MODE

    def on_enter_ASK_MODE(self):
        print("--- ESTADO: ASK_MODE ---")
        # Aquí el robot SE DETIENE. 
        # Ya no hay 'while listen()'. Espera a que la tableta mande señal.
        self.say("Configura el juego y pulsa iniciar.")

    def on_enter_SETUP_GAME(self):
        print("--- ESTADO: SETUP_GAME ---")
        # Transición rápida, la tableta hace el setup real instantáneamente.
        self.say("¡Listo!")
        self.setup_ok() 

    def on_enter_LOOP(self):
        print("--- ESTADO: LOOP (Juego Activo) ---")
        # El robot entra en modo "animador".
        self.say("¡A jugar! Encuentra las parejas.")
        # Se queda aquí hasta que la API dispare 'say_results' (Win)

    def on_enter_SAY_RESULT(self):
        print("--- ESTADO: SAY_RESULT ---")
        if self.winner_name and "Empate" not in self.winner_name:
             self.say(f"¡Juego terminado! Felicidades {self.winner_name}.")
        else:
             self.say("¡Juego terminado! Ha sido un empate.")
        
        time.sleep(2)
        self.say("Si quieren revancha, pulsen reiniciar.")

    def on_enter_ERROR_EXIT(self):
        self.say("Error en el sistema.")
        os._exit(0)

    # --- Reacciones (Sin cambio de estado) ---
    def react_match(self):
        if self.state == 'LOOP':
            self.say("¡Muy bien!")

# Instancia Global
juego = JuegoMemoria()

# ==========================================
# RUTAS API (El Puente Tableta -> FSM)
# ==========================================

@app.get("/", response_class=HTMLResponse)
async def index_page(request: Request):
    return templates.TemplateResponse("index.html", {"request": request})

@app.post("/api/start")
async def api_start():
    """Tableta avisa: Botón Iniciar presionado"""
    if juego.state == 'ASK_MODE':
        juego.mode_ok() # Dispara ASK_MODE -> SETUP_GAME
    return {"status": "ok"}

@app.post("/api/match")
async def api_match():
    """Tableta avisa: Encontraron pareja"""
    juego.react_match()
    return {"status": "ok"}

@app.post("/api/win")
async def api_win(data: GameData):
    """Tableta avisa: Juego Terminado"""
    juego.winner_name = data.winner
    if juego.state == 'LOOP':
        juego.say_results() # Dispara LOOP -> SAY_RESULT
    return {"status": "ok"}

@app.post("/api/reset")
async def api_reset():
    """Tableta avisa: Botón Reiniciar presionado"""
    if juego.state == 'SAY_RESULT' or juego.state == 'LOOP':
        juego.play_again() # Dispara SAY_RESULT -> ASK_MODE
    return {"status": "ok"}

# ==========================================
# MAIN
# ==========================================

if __name__ == "__main__":
    # Arrancar la lógica inicial del robot en hilo aparte
    def kickstart():
        time.sleep(5) # Esperar a uvicorn
        if juego.state == 'INIT':
            juego.on_enter_INIT()
            
    threading.Thread(target=kickstart, daemon=True).start()
    
    print("🚀 Servidor listo. Esperando a la tableta...")
    uvicorn.run(app, host="0.0.0.0", port=8001)