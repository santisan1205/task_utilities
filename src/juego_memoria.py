
"""

Flujo:
  INIT → INTRO → ASK_MODE → SETUP_GAME → LOOP → SAY_RESULT → ASK_MODE → ...
  (En cualquier estado: trigger global 'error' → ERROR_EXIT)

"""

from __future__ import annotations
import json, random, pathlib, time, os, threading
from typing import List, Dict, Any, Optional

import rospy
from transitions import Machine
from task_module import Task_module as TM


# ====================== Motor del juego (clase separada) ======================

class MemoryGame:
    """Lógica del juego (independiente de la FSM/voz)."""
    def __init__(self, mode: str, board: List[Dict[str, Any]], player_names: Optional[List[str]] = None):
        if mode not in ("solo", "versus"):
            raise ValueError("mode debe ser 'solo' o 'versus'")
        self.mode = mode
        self.board = board
        self.players = player_names or (["P1"] if mode == "solo" else ["P1", "P2"])
        if mode == "versus" and len(self.players) != 2:
            raise ValueError("En 'versus' deben ser 2 jugadores.")
        self.current_player_idx = 0
        self.scores = {name: 0 for name in self.players}
        self.moves = 0
        self.flips_buffer: List[int] = []
        self.finished = False

    @property
    def current_player(self) -> str:
        return self.players[self.current_player_idx]

    def game_over(self) -> bool:
        return all(c["state"] == "matched" for c in self.board)

    def flip(self, index: int) -> Dict[str, Any]:
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
            return {"ok": True, "action": "flip_first", "player": self.current_player, "index": index}

        if len(self.flips_buffer) == 2:
            i, j = self.flips_buffer
            c1, c2 = self.board[i], self.board[j]
            self.moves += 1

            if c1["pid"] == c2["pid"]:
                c1["state"] = c2["state"] = "matched"
                self.scores[self.current_player] += 1
                self.flips_buffer.clear()

                if self.game_over():
                    self.finished = True
                    return {"ok": True, "action": "match_and_finish", "i": i, "j": j,
                            "scores": self.scores, "moves": self.moves, "winner": self._winner()}
                return {"ok": True, "action": "match", "i": i, "j": j,
                        "scores": self.scores, "moves": self.moves,
                        "extra_turn": (self.mode == "versus")}
            else:
                c1["state"] = c2["state"] = "down"
                self.flips_buffer.clear()
                if self.mode == "versus":
                    self.current_player_idx = 1 - self.current_player_idx

                if self.game_over():
                    self.finished = True
                    return {"ok": True, "action": "mismatch_and_finish", "i": i, "j": j,
                            "scores": self.scores, "moves": self.moves, "winner": self._winner()}
                return {"ok": True, "action": "mismatch", "i": i, "j": j,
                        "scores": self.scores, "moves": self.moves}

        return {"ok": True, "action": "flip_extra", "index": index}

    def _winner(self) -> Optional[str]:
        if self.mode == "solo":
            return self.players[0]
        p1, p2 = self.players
        if self.scores[p1] > self.scores[p2]: return p1
        if self.scores[p2] > self.scores[p1]: return p2
        return None

    def report(self, category: str, language: str) -> Dict[str, Any]:
        return {
            "mode": self.mode, "players": self.players, "moves": self.moves,
            "scores": self.scores, "category": category, "language": language,
            "finished": self.finished, "winner": self._winner(),
        }




class JuegoMemoriaFSM(object):
    """
    FSM:
      INIT → INTRO → ASK_MODE → SETUP_GAME → LOOP → SAY_RESULT → ASK_MODE → ...
      (error) → ERROR_EXIT

    """
    # ====================== FSM con helpers “dentro de la clase” ======================

    STATES = [
        "INIT",
        "INTRO",
        "ASK_MODE",
        "SETUP_GAME",   
        "LOOP",
        "SAY_RESULT",
        "ERROR_EXIT",
    ]

    TRANSITIONS = [
        {"trigger": "start",        "source": "INIT",       "dest": "INTRO"},
        {"trigger": "go_ask_mode",  "source": "INTRO",      "dest": "ASK_MODE"},
        {"trigger": "mode_ok",      "source": "ASK_MODE",   "dest": "SETUP_GAME"},
        {"trigger": "setup_ok",     "source": "SETUP_GAME", "dest": "LOOP"},
        {"trigger": "say_results",  "source": "LOOP",       "dest": "SAY_RESULT"},
        {"trigger": "play_again",   "source": "SAY_RESULT", "dest": "ASK_MODE"},   # ciclo infinito
        {"trigger": "error",        "source": "*",          "dest": "ERROR_EXIT"}, # manejo global de errores
    ]

    def __init__(self, voice: bool = True):
        # Pepper
        self.tm = TM(speech=True, pytoolkit=True)
        self.tm.initialize_node("JUEGO_MEMORIA_FSM")
        self.tm.initialize_pepper()

        # Config/voz
        self.voice = voice
        self.tts_lang = "Spanish"
        self.stt_lang = "spa"
        self.max_retries = 3

        # Selecciones
        self.mode: Optional[str] = None
        self.category: Optional[str] = None
        self.language: str = "es"

        # Juego
        self.deck: Optional[Dict[str, Any]] = None
        self.board: Optional[List[Dict[str, Any]]] = None
        self.game: Optional[MemoryGame] = None

        # Carga config
        self.cfg = self._load_config("game_config.json")

        # FSM
        self.machine = Machine(model=self, states=STATES, transitions=TRANSITIONS, initial="INIT")

        # Watchdog ROS
        self._wd = threading.Thread(target=self._check_ros_shutdown, daemon=True)
        self._wd.start()

    def _load_config(self, path: str | pathlib.Path) -> Dict[str, Any]:
        p = pathlib.Path(path)
        data = json.loads(p.read_text(encoding="utf-8"))
        if "modes" not in data or "categories" not in data:
            raise ValueError("game_config.json debe tener 'modes' y 'categories'.")
        return data

    def _load_deck_json(self, path: str | pathlib.Path) -> Dict[str, Any]:
        p = pathlib.Path(path)
        data = json.loads(p.read_text(encoding="utf-8"))
        for k in ("category", "language", "cards"):
            if k not in data:
                raise ValueError(f"Deck inválido: falta '{k}' en {path}")
        cards = data["cards"]
        if len(cards) != 8:
            raise ValueError(f"Deck debe tener 8 ítems únicos; tiene {len(cards)} en {path}")
        seen = set()
        for c in cards:
            for k in ("id", "label", "image"):
                if k not in c:
                    raise ValueError(f"Carta inválida (falta '{k}'): {c}")
            if c["id"] in seen:
                raise ValueError(f"IDs repetidos: {c['id']}")
            seen.add(c["id"])
        return data

    def _build_board_from_deck(self, deck: Dict[str, Any], seed: Optional[int] = None) -> List[Dict[str, Any]]:
        duplicated: List[Dict[str, Any]] = []
        for c in deck["cards"]:
            duplicated.append({"uid": f"{c['id']}#1", "pid": c["id"], "label": c["label"], "image": c["image"], "state": "down"})
            duplicated.append({"uid": f"{c['id']}#2", "pid": c["id"], "label": c["label"], "image": c["image"], "state": "down"})
        rng = random.Random(seed)
        rng.shuffle(duplicated)
        return duplicated

    def _norm(self, text: str) -> str:
        if not text: return ""
        t = text.lower().strip()
        rep = {"ó":"o","á":"a","é":"e","í":"i","ú":"u"," uno contra uno ":" versus "," vs ":" versus "}
        for k,v in rep.items(): t = t.replace(k, v)
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
        if "objetos del aula" in t or "objetos" in t or "aula" in t or "salon" in t: return "objetos_del_aula"
        return None

    def _reset_round(self):
        """Limpia recursos de la ronda previa (para volver a ASK_MODE)."""
        self.category = None
        self.deck = None
        self.board = None
        self.game = None

    def _hard_exit(self, say_bye: bool = True):
        """Salida inmediata del proceso (sin estado FINISH)."""
        try:
            if say_bye:
                self.say("Hasta pronto.")
        finally:
            os._exit(os.EX_OK)

    # ------------------ Voz ------------------

    def say(self, text: str):
        self.tm.talk(text=text, language=self.tts_lang)

    def listen(self, seconds=4) -> str:
        return self.tm.speech2text_srv(seconds=seconds, lang=self.stt_lang) or ""

    # ------------------ ESTADOS ------------------

    def on_enter_INTRO(self):
        self.say("Hola. Vamos a jugar memoria. Debes voltear dos cartas iguales para ganar puntos.")
        time.sleep(0.3)
        self.say("Primero elegiremos el modo y la categoría.")
        self.go_ask_mode()

    def on_enter_ASK_MODE(self):
        retries = 0
        self.mode = None
        while retries < self.max_retries and self.mode is None:
            if self.voice:
                self.say("Di el modo: solo, o versus. También puedes decir: salir.")
                utter = self.listen(4)
                if "salir" in self._norm(utter):
                    self._hard_exit(); return
                m = self._match_mode(utter)
            else:
                m = "versus"  # default sin voz
            if m:
                self.mode = m
                self.say("Modo solo seleccionado." if m == "solo" else "Modo versus seleccionado.")
                self.mode_ok(); return
            retries += 1
            self.say("No te entendí. Repite: solo, o versus.")
        if self.mode is None:
            self.say("No logré entender el modo.")
            self.error()

    def on_enter_SETUP_GAME(self):
        """
        - Pide categoría (voz o default)
        - Carga deck
        - Construye tablero
        - Crea MemoryGame
        """
        try:
            # Categoría
            self.category = None
            retries = 0
            while retries < self.max_retries and self.category is None:
                if self.voice:
                    self.say("Elige categoría: animales, frutas, u objetos del aula. También puedes decir: salir.")
                    utter = self.listen(5)
                    if "salir" in self._norm(utter):
                        self._hard_exit(); return
                    c = self._match_category(utter)
                else:
                    c = "animales"
                if c:
                    self.category = c
                    bonitos = {"animales":"Animales","frutas":"Frutas","objetos_del_aula":"Objetos del aula"}
                    self.say(f"Categoría {bonitos[c]} seleccionada.")
                    break
                retries += 1
                self.say("No te entendí. Di: animales, frutas, u objetos del aula.")
            if self.category is None:
                self.say("No logré entender la categoría.")
                self.error(); return

            # Deck y tablero
            deck_path = pathlib.Path("decks") / f"{self.category}_es.json"
            self.deck = self._load_deck_json(deck_path)
            self.board = self._build_board_from_deck(self.deck, seed=None)

            # Juego
            players = ["Ana", "Luis"] if self.mode == "versus" else ["Tú"]
            self.game = MemoryGame(mode=self.mode, board=self.board, player_names=players)

            self.say(f"Iniciando partida. Modo {self.mode}. Categoría {self.category.replace('_',' ')}.")
            # Hook UI web:
            game_url = f"http://<TU_IP>:<PUERTO>/index.html?mode={self.mode}&category={self.category}&lang=es"
            print("[juego_memoria] Si tienes UI web, abrir:", game_url)

            self.setup_ok()
        except Exception as e:
            print("[juego_memoria] Error en SETUP_GAME:", e)
            self.say("Ocurrió un problema preparando la partida.")
            self.error()

    def on_enter_LOOP(self):
        """
        Se puede integrar UI/voz para voltear cartas:
          - UI: self.game.flip(idx)
          - Voz: 'voltea fila 2 columna 3' → idx → flip
        Para anunciar resultados, se llama self.say_results()
        """
        try:
            # DEMO mínima
            r1 = self.game.flip(0); r2 = self.game.flip(1)
            print("Jugada 1:", r1, r2)
            if r2.get("action") == "match": self.say("¡Acierto!")
            elif r2.get("action") == "mismatch": self.say("No coinciden. Siguiente turno.")

            r3 = self.game.flip(2); r4 = self.game.flip(3)
            print("Jugada 2:", r3, r4)

            # Anuncio
            self.say_results()
        except Exception as e:
            print("[juego_memoria] Error en LOOP:", e)
            self.error()

    def on_enter_SAY_RESULT(self):
        """Anuncia resultados y vuelve al estado ASK_MODE (bucle infinito)."""
        try:
            rep = self.game.report(category=self.deck["category"], language=self.deck["language"])
            print("Reporte:", rep)
            if rep["finished"]:
                if rep["winner"]:
                    self.say(f"Juego terminado. Ganador: {rep['winner']}. Puntajes: {rep['scores']}.")
                else:
                    self.say(f"Juego terminado. Empate. Puntajes: {rep['scores']}.")
            else:
                self.say(f"Partida en curso. Puntajes: {rep['scores']}.")

            # reset y preguntar de nuevo el modo
            self._reset_round()
            time.sleep(0.3)
            self.say("¿Jugamos otra vez? Vamos a seleccionar el modo.")
            self.play_again()
        except Exception as e:
            print("[juego_memoria] Error al anunciar resultados:", e)
            self.error()

    def on_enter_ERROR_EXIT(self):
        """Salida por error."""
        self.say("Ha ocurrido un error. Cerrando el juego.")
        os._exit(os.EX_OK)

    # ------------------ Infra ROS ------------------

    def _check_ros_shutdown(self):
        while not rospy.is_shutdown():
            time.sleep(0.1)
        print("[juego_memoria] ROS shutdown. Saliendo.")
        os._exit(os.EX_OK)

    def run(self):
        self.start()
        while not rospy.is_shutdown():
            time.sleep(0.1)


# ====================== MAIN ======================

if __name__ == "__main__":
    # voice=True usa STT/TTS en modo y categoría; False usa defaults
    jm = JuegoMemoriaFSM(voice=True)
    jm.run()
