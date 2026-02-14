#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
juego_memoria.py — Supervisor del Juego
Integración: FSM Original 
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

# --- Imports Robot (Modo Simulación si estás en PC) ---
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

# --- Red ---
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

# --- FastAPI ---
app = FastAPI()
script_dir = os.path.dirname(os.path.abspath(__file__))
templates = Jinja2Templates(directory=os.path.join(script_dir, "templates"))
app.mount("/static", StaticFiles(directory=os.path.join(script_dir, "static")), name="static")

app.add_middleware(
    CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"],
)

class GameData(BaseModel):
    winner: str = None

# ==========================================
# MÁQUINA DE ESTADOS (Supervisora)
# ==========================================

class JuegoMemoria(object):

    STATES = [
        "INIT", "INTRO", "ASK_MODE", "SETUP_GAME",
        "LOOP", "SAY_RESULT", "ERROR_EXIT"
    ]

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
        try:
            self.tm = TM(speech=True, pytoolkit=True)
            self.tm.initialize_node("JUEGO_MEMORIA_FSM")
            self.tm.initialize_pepper()
        except:
            pass

        self.tts_lang = "Spanish"
        self.winner_name = "" 

        self.machine = Machine(model=self, states=self.STATES, transitions=self.TRANSITIONS, initial="INIT")

    def say(self, text: str):
        print(f"🤖 Pepper: {text}")
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

    # --- ACCIONES DE ESTADO ---

    def on_enter_INIT(self):
        print("--- ESTADO: INIT ---")
        self.show_tablet()
        time.sleep(2)
        self.start()

    def on_enter_INTRO(self):
        print("--- ESTADO: INTRO ---")
        self.say("Hola. Soy Pepper. Vamos a jugar memoria.")
        self.go_ask_mode()

    def on_enter_ASK_MODE(self):
        print("--- ESTADO: ASK_MODE ---")
        self.say("Configura la partida en mi pantalla y pulsa Iniciar.")
        # Se queda aquí esperando evento /api/start

    def on_enter_SETUP_GAME(self):
        print("--- ESTADO: SETUP ---")
        self.say("Preparando tablero...")
        self.setup_ok()

    def on_enter_LOOP(self):
        print("--- ESTADO: LOOP (Jugando) ---")
        self.say("¡A jugar! Encuentra las parejas.")
        # Se queda aquí esperando evento /api/win

    def on_enter_SAY_RESULT(self):
        print("--- ESTADO: RESULTADOS ---")
        if "Empate" in self.winner_name:
             self.say("Ha sido un empate. ¡Qué reñido!")
        else:
             self.say(f"¡Felicidades! Ha ganado {self.winner_name}.")
        time.sleep(1)
        self.say("Si quieren jugar de nuevo, pulsen Reiniciar.")

    def on_enter_ERROR_EXIT(self):
        self.say("Error.")
        os._exit(0)

    def react_match(self):
        if self.state == 'LOOP':
            import random
            frases = ["¡Bien hecho!", "¡Eso es!", "¡Genial!"]
            self.say(random.choice(frases))

juego = JuegoMemoria()

# ==========================================
# API ENDPOINTS (Conectan JS con Python)
# ==========================================

@app.get("/", response_class=HTMLResponse)
async def index_page(request: Request):
    return templates.TemplateResponse("index.html", {"request": request})

@app.post("/api/start")
async def api_start():
    if juego.state == 'ASK_MODE':
        juego.mode_ok()
    return {"status": "ok"}

@app.post("/api/match")
async def api_match():
    juego.react_match()
    return {"status": "ok"}

@app.post("/api/win")
async def api_win(data: GameData):
    juego.winner_name = data.winner
    if juego.state == 'LOOP':
        juego.say_results()
    return {"status": "ok"}

@app.post("/api/reset")
async def api_reset():
    if juego.state == 'SAY_RESULT' or juego.state == 'LOOP':
        juego.play_again()
    return {"status": "ok"}

if __name__ == "__main__":
    def kickstart():
        time.sleep(5)
        if juego.state == 'INIT':
            juego.on_enter_INIT()
    threading.Thread(target=kickstart, daemon=True).start()
    uvicorn.run(app, host="0.0.0.0", port=8001) 