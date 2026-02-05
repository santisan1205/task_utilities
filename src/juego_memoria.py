#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
juego_memoria.py — Juego de memoria para Pepper
  INIT → INTRO → ASK_MODE → SETUP_GAME → LOOP → SAY_RESULT → ASK_MODE → ...
  (En cualquier estado: trigger 'error' → ERROR_EXIT)
"""

from __future__ import annotations
import json, random, time, os, threading
from typing import List, Dict, Any, Optional

import rospy
from transitions import Machine
from task_module import Task_module as TM


class JuegoMemoria(object):
    """
    Clase única que contiene:
    - FSM (states + transitions)
    - Voz (say/listen)
    - Carga de config/decks desde el mismo directorio
    - Lógica del juego (flip/puntajes/ganador) SIN clases externas
    """

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

    def __init__(self, voice: bool = True):
        # Inicializar Pepper
        self.tm = TM(speech=True, pytoolkit=True)
        self.tm.initialize_node("JUEGO_MEMORIA_FSM")
        self.tm.initialize_pepper()

        # Configuración de voz
        self.voice = voice
        self.tts_lang = "Spanish"
        self.stt_lang = "spa"
        self.max_retries = 3

        # Variables del juego
        self.mode: Optional[str] = None
        self.category: Optional[str] = None
        self.language: str = "es"

        self.cfg: Dict[str, Any] = self._load_config("game_config.json")
        self.deck: Optional[Dict[str, Any]] = None
        self.board: Optional[List[Dict[str, Any]]] = None
        self.players: List[str] = []
        self.current_player_idx: int = 0
        self.scores: Dict[str, int] = {}
        self.moves: int = 0
        self.flips_buffer: List[int] = []
        self.finished: bool = False

        # FSM
        self.machine = Machine(model=self, states=self.STATES, transitions=self.TRANSITIONS, initial="INIT")

        # Watchdog ROS
        self._wd = threading.Thread(target=self._check_ros_shutdown, daemon=True)
        self._wd.start()

    # ---------- Voz ----------
    def say(self, text: str):
        """Hablar con Pepper."""
        self.tm.talk(text=text, language=self.tts_lang)

    def listen(self, seconds: int = 4) -> str:
        """Escuchar entrada por voz."""
        return self.tm.speech2text_srv(seconds=seconds, lang=self.stt_lang) or ""

    # ---------- Carga segura de archivos ----------
    def _get_local_path(self, filename: str) -> str:
        """Devuelve la ruta absoluta de un archivo en la misma carpeta del script."""
        base_dir = os.path.dirname(__file__)
        return os.path.join(base_dir, filename)

    def _load_config(self, filename: str = "game_config.json") -> Dict[str, Any]:
        """Carga y valida la configuración global del juego."""
        file_path = self._get_local_path(filename)
        if not os.path.exists(file_path):
            raise FileNotFoundError(f"No se encontró {filename} en {os.path.dirname(__file__)}")
        with open(file_path, "r", encoding="utf-8") as f:
            data = json.load(f)
        if "modes" not in data or "categories" not in data:
            raise ValueError("game_config.json debe tener 'modes' y 'categories'.")
        return data

    def _load_deck_json(self, filename: str) -> Dict[str, Any]:
        """Carga un mazo JSON de la misma carpeta que el script."""
        file_path = self._get_local_path(filename)
        if not os.path.exists(file_path):
            raise FileNotFoundError(f"No se encontró el mazo: {filename}")
        with open(file_path, "r", encoding="utf-8") as f:
            data = json.load(f)
        for k in ("category", "language", "cards"):
            if k not in data:
                raise ValueError(f"Deck inválido: falta '{k}' en {filename}")
        return data

    def _build_board_from_deck(self, deck: Dict[str, Any], seed: Optional[int] = None) -> List[Dict[str, Any]]:
        """Duplica las cartas para crear pares y las baraja."""
        duplicated: List[Dict[str, Any]] = []
        for c in deck["cards"]:
            duplicated.extend([
                {"uid": f"{c['id']}#1", "pid": c["id"], "label": c["label"], "image": c["image"], "state": "down"},
                {"uid": f"{c['id']}#2", "pid": c["id"], "label": c["label"], "image": c["image"], "state": "down"}
            ])
        random.Random(seed).shuffle(duplicated)
        return duplicated

    # ---------- Normalización y matching ----------
    def _norm(self, text: str) -> str:
        if not text: return ""
        t = text.lower().strip()
        rep = {"ó":"o","á":"a","é":"e","í":"i","ú":"u"," uno contra uno ":" versus "," vs ":" versus "}
        for k, v in rep.items():
            t = t.replace(k, v)
        return t

    def _match_mode(self, text: str) -> Optional[str]:
        t = self._norm(text)
        if "solo" in t: return "solo"
        if "versus" in t or "vs" in t: return "versus"
        return None

    def _match_category(self, text: str) -> Optional[str]:
        t = self._norm(text)
        if "animales" in t or "animal" in t:  return "animales"
        if "fruta" in t or "frutas" in t:     return "frutas"
        if "objetos" in t or "aula" in t or "salon" in t: return "objetos_del_aula"
        return None

    # ---------- Utilidades de control ----------
    def _reset_round(self):
        """Limpia recursos de la ronda previa (para volver a ASK_MODE)."""
        self.category = None
        self.deck = None
        self.board = None
        self.players = []
        self.current_player_idx = 0
        self.scores = {}
        self.moves = 0
        self.flips_buffer = []
        self.finished = False

    def _hard_exit(self):
        """Salida inmediata del programa."""
        try:
            self.say("Hasta pronto.")
        finally:
            os._exit(os.EX_OK)

    # ---------- Motor del juego ----------
    def _flip(self, index: int) -> Dict[str, Any]:
        """Voltea cartas y resuelve si hay match."""
        if self.finished:
            return {"ok": False, "reason": "finished"}
        if not (0 <= index < len(self.board)):
            return {"ok": False, "reason": "invalid_index"}

        card = self.board[index]
        if card["state"] != "down":
            return {"ok": False, "reason": "card_not_down"}

        card["state"] = "up"
        self.flips_buffer.append(index)

        if len(self.flips_buffer) == 1:
            return {"ok": True, "action": "flip_first", "index": index}

        if len(self.flips_buffer) == 2:
            i, j = self.flips_buffer
            c1, c2 = self.board[i], self.board[j]
            self.moves += 1

            if c1["pid"] == c2["pid"]:
                c1["state"] = c2["state"] = "matched"
                self.scores[self.players[self.current_player_idx]] += 1
                self.flips_buffer.clear()
                if all(c["state"] == "matched" for c in self.board):
                    self.finished = True
                    return {"ok": True, "action": "match_and_finish", "winner": self._winner()}
                return {"ok": True, "action": "match"}
            else:
                c1["state"] = c2["state"] = "down"
                self.flips_buffer.clear()
                if self.mode == "versus":
                    self.current_player_idx = 1 - self.current_player_idx
                return {"ok": True, "action": "mismatch"}

        return {"ok": True, "action": "flip_extra"}

    def _winner(self) -> Optional[str]:
        if self.mode == "solo":
            return self.players[0]
        if self.scores[self.players[0]] > self.scores[self.players[1]]:
            return self.players[0]
        elif self.scores[self.players[1]] > self.scores[self.players[0]]:
            return self.players[1]
        else:
            return None

    # ---------- ESTADOS ----------
    def on_enter_INTRO(self):
        self.say("Hola. Vamos a jugar memoria. Debes voltear dos cartas iguales para ganar puntos.")
        time.sleep(0.3)
        self.say("Primero elegiremos el modo y la categoría.")
        self.go_ask_mode()

    def on_enter_ASK_MODE(self):
        retries = 0
        self.mode = None
        while retries < self.max_retries and not self.mode:
            if self.voice:
                self.say("Di el modo: solo o versus. También puedes decir salir.")
                utter = self.listen(4)
                if "salir" in self._norm(utter):
                    self._hard_exit(); return
                self.mode = self._match_mode(utter)
            else:
                self.mode = "versus"
            if self.mode:
                self.say("Modo seleccionado: " + self.mode)
                self.mode_ok(); return
            retries += 1
            self.say("No te entendí, repite.")
        self.error()

    def on_enter_SETUP_GAME(self):
        """Pide categoría, carga deck y prepara tablero."""
        try:
            retries = 0
            self.category = None
            while retries < self.max_retries and not self.category:
                if self.voice:
                    self.say("Elige categoría: animales, frutas u objetos del aula. También puedes decir salir.")
                    utter = self.listen(5)
                    if "salir" in self._norm(utter):
                        self._hard_exit(); return
                    self.category = self._match_category(utter)
                else:
                    self.category = "animales"
                if not self.category:
                    retries += 1
                    self.say("No te entendí. Repite por favor.")
            if not self.category:
                self.error(); return

            self.say(f"Cargando cartas de {self.category}.")
            self.deck = self._load_deck_json(f"{self.category}_es.json")
            self.board = self._build_board_from_deck(self.deck)

            self.players = ["Ana", "Luis"] if self.mode == "versus" else ["Tú"]
            self.scores = {p: 0 for p in self.players}
            self.moves = 0
            self.finished = False

            self.say(f"Iniciando partida. Modo {self.mode}, categoría {self.category}.")
            self.setup_ok()
        except Exception as e:
            print("[juego_memoria] Error en SETUP_GAME:", e)
            self.say("Ocurrió un problema cargando el mazo.")
            self.error()

    def on_enter_LOOP(self):
        """Simulación temporal de jugadas."""
        try:
            r1 = self._flip(0); r2 = self._flip(1)
            print("Jugada 1:", r1, r2)
            if r2.get("action") == "match": self.say("¡Acierto!")
            elif r2.get("action") == "mismatch": self.say("No coinciden.")

            r3 = self._flip(2); r4 = self._flip(3)
            print("Jugada 2:", r3, r4)
            self.say_results()
        except Exception as e:
            print("[juego_memoria] Error en LOOP:", e)
            self.error()

    def on_enter_SAY_RESULT(self):
        """Anuncia resultados y vuelve a preguntar modo."""
        try:
            winner = self._winner()
            if self.finished:
                if winner:
                    self.say(f"Juego terminado. Ganador: {winner}. Puntajes: {self.scores}.")
                else:
                    self.say(f"Juego terminado. Empate. Puntajes: {self.scores}.")
            else:
                self.say(f"Partida en curso. Puntajes: {self.scores}.")
            self._reset_round()
            self.say("¿Jugamos otra vez? Selecciona el modo.")
            self.play_again()
        except Exception as e:
            print("[juego_memoria] Error al anunciar resultados:", e)
            self.error()

    def on_enter_ERROR_EXIT(self):
        """Manejo global de errores."""
        self.say("Ha ocurrido un error. Cerrando el juego.")
        os._exit(os.EX_OK)

    # ---------- ROS ----------
    def _check_ros_shutdown(self):
        while not rospy.is_shutdown():
            time.sleep(0.1)
        os._exit(os.EX_OK)

    def run(self):
        self.start()
        while not rospy.is_shutdown():
            time.sleep(0.1)


# ---------- MAIN ----------
if __name__ == "__main__":
    jm = JuegoMemoria(voice=True)
    jm.run()

