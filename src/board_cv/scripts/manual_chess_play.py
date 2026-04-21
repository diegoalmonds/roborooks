#!/usr/bin/env python3
"""
manual_chess_play.py
Fallback script — same Stream Deck / stdin override logic as cv_chess_play.py
but with NO RealSense camera required. Useful for demos or when CV is unavailable.
"""

import cv2
import json
import numpy as np
import chess
import chess.engine
import chess.svg
from PIL import Image
import io
import random
import os
import sys
import cairosvg
import threading
import queue
import rospy

from board_cv.msg import Move

rospy.init_node("board_vision")
move_pub = rospy.Publisher("/ai_move", Move, queue_size=10)

# === CONFIG ===
CALIB_JSON = "sqdict.json"
ENGINE_PATH = r"/usr/games/stockfish"

# === ENGINE ===
if not os.path.exists(ENGINE_PATH):
    print(f"[ERROR] Engine file not found: {ENGINE_PATH}")
    sys.exit(1)

engine = chess.engine.SimpleEngine.popen_uci(ENGINE_PATH)
print(f"[INFO] Stockfish started from {ENGINE_PATH}")

# === LOAD JSON ===
if not os.path.exists(CALIB_JSON):
    print(f"[ERROR] Calibration file not found: {CALIB_JSON}")
    engine.quit()
    sys.exit(1)

with open(CALIB_JSON, "r") as f:
    sq_points = json.load(f)
print(f"[INFO] Loaded {len(sq_points)} squares from {CALIB_JSON}")

files = 'abcdefgh'
ranks = '12345678'

# =============================================================================
# === STDIN THREAD (Stream Deck input) ========================================
# =============================================================================

stdin_queue = queue.Queue()

def stdin_reader():
    while True:
        try:
            line = sys.stdin.readline()
            if line:
                stdin_queue.put(line.strip())
        except Exception:
            break

stdin_thread = threading.Thread(target=stdin_reader, daemon=True)
stdin_thread.start()

def get_stdin_line():
    try:
        return stdin_queue.get_nowait()
    except queue.Empty:
        return None

# =============================================================================
# === MANUAL OVERRIDE STATE MACHINE ===========================================
# =============================================================================

override_state = "idle"
override_buf = {}

def reset_override():
    global override_state, override_buf
    override_state = "idle"
    override_buf = {}
    print("[OVERRIDE] Cancelled.")

def override_prompt():
    prompts = {
        "await_type":   "[OVERRIDE] Move type — 'r' regular / 'c' castle / 'p' promotion:",
        "await_from_f": "[OVERRIDE] FROM file (a-h):",
        "await_from_r": "[OVERRIDE] FROM rank (1-8):",
        "await_to_f":   "[OVERRIDE] TO file (a-h):",
        "await_to_r":   "[OVERRIDE] TO rank (1-8):",
        "await_promo":  "[OVERRIDE] Promotion piece — q / r / b / n:",
        "await_castle": "[OVERRIDE] Castle side — 'k' kingside / 'q' queenside:",
    }
    msg = prompts.get(override_state)
    if msg:
        print(msg)

def handle_override_token(token, board):
    global override_state, override_buf

    token = token.strip().lower()

    if token == 'x':
        reset_override()
        return None

    if override_state == "await_type":
        if token == 'r':
            override_state = "await_from_f"
        elif token == 'c':
            override_state = "await_castle"
        elif token == 'p':
            override_state = "await_from_f"
            override_buf['promotion'] = True
        else:
            print(f"[OVERRIDE] Unknown type '{token}'. Use r/c/p.")
        override_prompt()
        return None

    if override_state == "await_castle":
        is_white = board.turn == chess.WHITE
        if token == 'k':
            uci = "e1g1" if is_white else "e8g8"
        elif token == 'q':
            uci = "e1c1" if is_white else "e8c8"
        else:
            print(f"[OVERRIDE] Unknown castle side '{token}'. Use k/q.")
            override_prompt()
            return None
        try:
            mv = chess.Move.from_uci(uci)
            if mv in board.legal_moves:
                override_state = "idle"; override_buf = {}
                return mv
            else:
                print(f"[OVERRIDE] Castle {uci} not legal right now.")
                reset_override(); return None
        except Exception as e:
            print(f"[OVERRIDE] Castle error: {e}")
            reset_override(); return None

    if override_state == "await_from_f":
        if token in 'abcdefgh' and len(token) == 1:
            override_buf['from_f'] = token
            override_state = "await_from_r"
        else:
            print(f"[OVERRIDE] Invalid file '{token}'.")
        override_prompt(); return None

    if override_state == "await_from_r":
        if token in '12345678' and len(token) == 1:
            override_buf['from_r'] = token
            override_state = "await_to_f"
        else:
            print(f"[OVERRIDE] Invalid rank '{token}'.")
        override_prompt(); return None

    if override_state == "await_to_f":
        if token in 'abcdefgh' and len(token) == 1:
            override_buf['to_f'] = token
            override_state = "await_to_r"
        else:
            print(f"[OVERRIDE] Invalid file '{token}'.")
        override_prompt(); return None

    if override_state == "await_to_r":
        if token in '12345678' and len(token) == 1:
            override_buf['to_r'] = token
            if override_buf.get('promotion'):
                override_state = "await_promo"
                override_prompt(); return None
            else:
                uci = (override_buf['from_f'] + override_buf['from_r'] +
                       override_buf['to_f']   + override_buf['to_r'])
                return _finalise_uci(uci, board)
        else:
            print(f"[OVERRIDE] Invalid rank '{token}'.")
        override_prompt(); return None

    if override_state == "await_promo":
        if token in ('q', 'r', 'b', 'n') and len(token) == 1:
            uci = (override_buf['from_f'] + override_buf['from_r'] +
                   override_buf['to_f']   + override_buf['to_r'] + token)
            return _finalise_uci(uci, board)
        else:
            print(f"[OVERRIDE] Invalid promotion piece '{token}'.")
        override_prompt(); return None

    return None

def _finalise_uci(uci, board):
    global override_state, override_buf
    try:
        mv = chess.Move.from_uci(uci)
        if mv in board.legal_moves:
            override_state = "idle"; override_buf = {}
            print(f"[OVERRIDE] Move accepted: {uci}")
            return mv
        else:
            print(f"[OVERRIDE] Move {uci} is not legal. Override cancelled.")
            reset_override(); return None
    except Exception as e:
        print(f"[OVERRIDE] Invalid move '{uci}': {e}")
        reset_override(); return None

# =============================================================================
# === HELPERS =================================================================
# =============================================================================

def show_board(board, last_move=None):
    svg = chess.svg.board(board=board, lastmove=last_move, coordinates=True, size=450)
    png_data = cairosvg.svg2png(bytestring=svg.encode('utf-8'))
    img = Image.open(io.BytesIO(png_data))
    img_cv = cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)
    cv2.imshow("Board State", img_cv)
    cv2.waitKey(1)

def publish_move(move_lan, board):
    ai_move = Move()
    ai_move.is_checkmate = board.is_checkmate()

    if 'O' in move_lan:
        ai_move.move_type = "castle"
        is_white = board.turn == chess.WHITE
        if move_lan == "O-O":
            ai_move.from_square = "e1" if is_white else "e8"
            ai_move.from_piece = "K"
            ai_move.to_square = "g1" if is_white else "g8"
            ai_move.to_piece = ""
            move_pub.publish(ai_move)
            ai_move.from_square = "h1" if is_white else "h8"
            ai_move.from_piece = "R"
            ai_move.to_square = "f1" if is_white else "f8"
            ai_move.to_piece = ""
            move_pub.publish(ai_move)
        else:
            ai_move.from_square = "e1" if is_white else "e8"
            ai_move.from_piece = "K"
            ai_move.to_square = "c1" if is_white else "c8"
            ai_move.to_piece = ""
            move_pub.publish(ai_move)
            ai_move.from_square = "a1" if is_white else "a8"
            ai_move.from_piece = "R"
            ai_move.to_square = "d1" if is_white else "d8"
            ai_move.to_piece = ""
            move_pub.publish(ai_move)
        return

    elif 'x' in move_lan:
        ai_move.move_type = "capture"
        from_part, to_part = move_lan.split('x')
        ai_move.from_square = from_part[1:3] if len(from_part) > 2 else from_part
        ai_move.from_piece = from_part[0] if len(from_part) > 2 else "P"
        ai_move.to_square = to_part[0:2]
        check_piece = board.piece_at(chess.parse_square(ai_move.to_square))
        ai_move.to_piece = check_piece.symbol().upper() if check_piece else ""
        if '=' in move_lan:
            ai_move.promotion_piece = move_lan.split('=')[1][0]

    elif '=' in move_lan:
        ai_move.move_type = "promotion"
        from_part, rest = move_lan.split('-')
        to_part, promo = rest.split('=')
        ai_move.from_square = from_part[1:3] if len(from_part) > 2 else from_part
        ai_move.from_piece = "P"
        ai_move.to_square = to_part[0:2]
        ai_move.to_piece = ""
        ai_move.promotion_piece = promo[0]

    elif '-' in move_lan:
        ai_move.move_type = "move"
        from_part, to_part = move_lan.split('-')
        ai_move.from_square = from_part[1:3] if len(from_part) > 2 else from_part
        ai_move.from_piece = from_part[0] if len(from_part) > 2 else "P"
        ai_move.to_square = to_part[0:2]
        check_piece = board.piece_at(chess.parse_square(ai_move.to_square))
        ai_move.to_piece = check_piece.symbol().upper() if check_piece else ""

    ai_move.notation = move_lan
    move_pub.publish(ai_move)

# =============================================================================
# === GAME STATE ==============================================================
# =============================================================================

board = chess.Board()
last_move = None
comp_turn = False
move_history = []

print("[INFO] Stream Deck / keyboard commands:")
print("  f = enter move  |  u = undo  |  U = undo 2  |  q = quit")
print("  During move entry:  x = cancel at any time")
show_board(board)

# =============================================================================
# === MAIN LOOP ===============================================================
# =============================================================================

try:
    while not board.is_game_over():
        key = cv2.waitKey(50) & 0xFF   # 50ms poll — no camera so we can block a bit longer
        token = get_stdin_line()

        # --- Top-level commands ---
        if (token == 'q' or key == ord('q')) and override_state == "idle":
            print("[INFO] Quitting.")
            break

        if token == 'u' or key == ord('u'):
            if override_state != "idle":
                reset_override()
            elif move_history:
                mv = move_history.pop()
                board.pop()
                print(f"[UNDO] Removed last move: {mv}")
                show_board(board)
            else:
                print("[INFO] No moves to undo.")
            token = None

        if token == 'U' or key == ord('U'):
            if override_state != "idle":
                reset_override()
            elif len(move_history) >= 2:
                mv2 = move_history.pop()
                mv1 = move_history.pop()
                board.pop(); board.pop()
                print(f"[UNDO] Removed last 2 moves: {mv1}, {mv2}")
                show_board(board)
            else:
                print("[INFO] Not enough moves to undo 2 times.")
            token = None

        # --- Trigger override ---
        if (token == 'f' or key == ord('f')) and override_state == "idle":
            override_state = "await_type"
            print("[OVERRIDE] Manual move entry activated.")
            override_prompt()
            token = None

        # --- Feed token into state machine ---
        if token and override_state != "idle":
            completed_move = handle_override_token(token, board)
            if completed_move is not None:
                lan = board.lan(completed_move)
                publish_move(lan, board)
                board.push(completed_move)
                move_history.append(completed_move)
                last_move = completed_move
                print(f"[YOU] Played: {completed_move.uci()} ({lan})")
                show_board(board, last_move)
                comp_turn = True

        # --- Computer turn ---
        if comp_turn and override_state == "idle":
            result = engine.play(board, chess.engine.Limit(time=random.uniform(0.4, 0.9)))
            mv = result.move
            lan = board.lan(mv)
            print(f"[AI] Computer played: {mv.uci()} ({lan})")
            publish_move(lan, board)
            board.push(mv)
            move_history.append(mv)
            last_move = mv
            show_board(board, last_move)
            comp_turn = False

    print("[INFO] Game over.")

finally:
    cv2.destroyAllWindows()
    engine.quit()