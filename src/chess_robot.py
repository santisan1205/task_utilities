#!/usr/bin/env python3
import chess
from transitions import Machine
from task_module import Task_module as tm
import ConsoleFormatter
import threading
import rospy
import os
#pip install chess


class Chess(object):
    def __init__(self):
        self.consoleFormatter=ConsoleFormatter.ConsoleFormatter()
        self.task_name = "CHESS"
        self.hearing = True
        self.new_game = False

        states =  ["CHESS", "INIT", "WAIT4GUEST"]
        self.tm = tm(perception = False ,speech=True, manipulation=False, pytoolkit=True)
        self.tm.initialize_node(self.task_name)
        
        #Definir transiciones
        transitions = [
            {"trigger": "start", "source": "CHESS", "dest": "INIT"},
            {"trigger": "begin", "source": "INIT", "dest": "WAIT4GUEST"},
            {"trigger": "newGame", "source": "WAIT4GUEST", "dest": "CHESS"},
            {"trigger": "finish", "source": "CHESS", "dest": "WAIT4GUEST"},
        ]

        self.machine = Machine(model=self, states=states, transitions=transitions, initial='CHESS')
        rospy_check = threading.Thread(target=self.check_rospy)
        rospy_check.start()

    def on_enter_INIT(self):
        print(self.consoleFormatter.format("Inicializacion del task: "+self.task_name, "HEADER"))
        self.tm.initialize_pepper()
        
        #self.tm.enable_breathing_service() 
        
        self.begin()

    def on_enter_WAIT4GUEST(self):
        print(self.consoleFormatter.format("WAIT4GUEST", "HEADER"))
        #TOCA CABEZA
        
        while not self.tm.wait_for_head_touch(timeout=20, message="Toca mi cabeza para jugar", message_interval=5, language="Spanish"):
            self.new_game = True
        self.tm.motion_tools_service()
        self.board = chess.Board()
        
        self.mate = False

        # Establecer conexion con stockFish

        self.tm.talk("Ya estoy lista para jugar!", "Spanish", wait=True)
        self.newGame()

    def on_enter_CHESS(self):
        print(self.consoleFormatter.format("CHESS", "HEADER"))

        while not self.mate:
            #Recibir FEN de perception
            fen_actual = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1"
            #Calcular movimiento a hacer con stockFISH
            # 2 casos
                # 1. StockFish retorna un fen
                # 2. StockFIsh retorna un SAN (E4 que corresponde a mover e2 a e4)

            #Asumo caso de FEN
            fen_nuevo = "rnbqkbnr/pppppppp/8/8/4P3/8/PPPP1PPP/RNBQKBNR b KQkq e3 0 1"

            move = self.detect_move_from_fens(fen_actual, fen_nuevo)
            move_text = self.movement(move, fen_actual)

            self.tm.talk(move_text, "Spanish",animated=False, wait=True)
            
            #QUITAR OBVIAMENTE
            self.mate = True
            
        self.new_game= False
        self.finish()


    def movement(self, move, fen_actual,language="es"):
        board = chess.Board(fen_actual)
        
        chess_move = board.parse_san(move)
        
        # Get data before move is executed
        is_capture = board.is_capture(chess_move)
        target_square = chess.square_name(chess_move.to_square)
        captured_piece = None
        
        if is_capture:
            # Determine the captured piece
            captured_square = chess_move.to_square
            if board.is_en_passant(chess_move):  # Handle en passant
                # For en passant, the captured pawn is on a different square
                ep_square = chess_move.to_square + (-8 if board.turn else 8)
                captured_piece = board.piece_at(ep_square)
            else:
                captured_piece = board.piece_at(captured_square)
        
        # Get the moving piece
        piece = board.piece_at(chess_move.from_square)
        
        # Execute the move
        board.push(chess_move)
        
        # Translate the move
        translation = self.translate_move(move, piece, is_capture, target_square, captured_piece, 
                                    self.board.is_check(), self.board.is_checkmate(), language)
        
        return translation

    def detect_move_from_fens(self ,fenActual: str, fenNuevo: str) -> str:
        """
        Given two FEN strings (before and after a move), return the move that was made.

        Returns the move in SAN (Standard Algebraic Notation), e.g., "e4", "Nf3", etc.
        If move is not found, returns an empty string.
        """
        board_before = chess.Board(fenActual)
        board_after = chess.Board(fenNuevo)

        for move in board_before.legal_moves:
            board_copy = board_before.copy()
            board_copy.push(move)

            if board_copy.board_fen() == board_after.board_fen():
                return board_before.san(move)  # Return the move in SAN format

        return "" # When no move match

    def translate_move(self, move, piece, is_capture, target_square, captured_piece, is_check, is_checkmate, language):
        """
        Create a natural language description of a chess move in first person perspective
        
        Args:
            move: Original move in algebraic notation
            piece: The chess piece that moved
            is_capture: Whether the move was a capture
            target_square: Destination square
            captured_piece: The piece that was captured (if any)
            is_check: Whether the move gives check
            is_checkmate: Whether the move gives checkmate
            language: Target language
            
        Returns:
            String description of the move
        """
        # Dictionary for piece names in Spanish
        piece_names_es = {
            'K': 'Rey',
            'Q': 'Reina',
            'R': 'Torre',
            'B': 'Alfil',
            'N': 'Caballo',
            'P': 'Peón'
        }
        
        # Handle castling
        if move == "O-O":
            if language == "es":
                return "Realizo enroque corto"
            
        if move == "O-O-O":
            if language == "es":
                return "Realizo enroque largo"
        
        # Check for check or checkmate
        check_str = ""
        if is_checkmate:
            check_str = " dando jaque mate"
            self.mate = True
        elif is_check:
            check_str = " dando jaque"
        
        # Create the translation
        if language == "es":
            # Get the piece name based on its type (K, Q, R, etc.)
            piece_type = piece.symbol().upper()  # Just get the piece type (K, Q, R, etc.)
            piece_name = piece_names_es.get(piece_type, piece_type)
            
            captured_piece_str = ""
            if captured_piece:
                captured_type = captured_piece.symbol().upper()
                captured_name = piece_names_es.get(captured_type, captured_type)
                captured_piece_str = f" ({captured_name})"
            
            if is_capture:
                translation = f"Muevo mi {piece_name.lower()} y capturo en {target_square}{captured_piece_str}{check_str}"
            else:
                translation = f"Muevo mi {piece_name.lower()} a {target_square}{check_str}"
        
        return translation

    def check_rospy(self):
        #Termina todos los procesos al cerrar el nodo
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
        print(self.consoleFormatter.format("Shutting down", "FAIL"))
        os._exit(os.EX_OK)

    def run(self):
        while not rospy.is_shutdown():
            self.start()
    
if __name__ == "__main__":
    sm = Chess()
    sm.run()
    rospy.spin()
