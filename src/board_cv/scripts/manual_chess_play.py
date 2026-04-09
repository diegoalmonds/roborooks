#!/usr/bin/env python3

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
import time
import rospy

from board_cv.msg import Move

rospy.init_node("board_vision")
move_pub = rospy.Publisher("/ai_move", Move, queue_size=10)

# === CONFIG ===
CALIB_JSON = "sqdict.json"
ENGINE_PATH = r"/usr/games/stockfish"
BOARD_ORIENTATION = "TOP"  # "TOP", "BOTTOM", "SIDE_L", "SIDE_R"

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

# === ORIENTATION ===
files = 'abcdefgh'
ranks = '12345678'

def remap_square(square_name: str) -> str:
    f = square_name[0]
    r = square_name[1]
    fi = files.index(f)
    ri = ranks.index(r)
    if BOARD_ORIENTATION == "TOP":
        return square_name
    elif BOARD_ORIENTATION == "BOTTOM":
        return f"{files[7 - fi]}{ranks[7 - ri]}"
    elif BOARD_ORIENTATION == "SIDE_L":
        return f"{files[ri]}{ranks[7 - fi]}"
    elif BOARD_ORIENTATION == "SIDE_R":
        return f"{files[7 - ri]}{ranks[fi]}"
    else:
        return square_name

# === HELPERS ===
def show_board(board, last_move=None):
    svg = chess.svg.board(board=board, lastmove=last_move, coordinates=True, size=450)
    png_data = cairosvg.svg2png(bytestring=svg.encode('utf-8'))
    img = Image.open(io.BytesIO(png_data))
    img_cv = cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)
    cv2.imshow("Board State", img_cv)
    cv2.waitKey(1)

def publish_robot_move(move, board):
    ai_move = Move()
    ai_move.is_checkmate = board.is_checkmate()
    if 'O' in move: # castle (PROBABLY BROKEN BC LAN NOTATION FOR CASTLE IS DIFFERENT)
        ai_move.move_type = "castle"
        if move == "O-O":
            ai_move.from_square = "e1" if board.turn == chess.WHITE else "e8"
            ai_move.from_piece = "K"
            ai_move.to_square = "g1" if board.turn == chess.WHITE else "g8"
            ai_move.to_piece = ""
            move_pub.publish(ai_move) # king move first
            ai_move.from_square = "h1" if board.turn == chess.WHITE else "h8"
            ai_move.from_piece = "R"
            ai_move.to_square = "f1" if board.turn == chess.WHITE else "f8"
            ai_move.to_piece = ""
            move_pub.publish(ai_move) # then rook move
            return
        else:
            ai_move.from_square = "e1" if board.turn == chess.WHITE else "e8"
            ai_move.from_piece = "K"
            ai_move.to_square = "c1" if board.turn == chess.WHITE else "c8"
            ai_move.to_piece = ""
            move_pub.publish(ai_move) # king move first
            ai_move.from_square = "a1" if board.turn == chess.WHITE else "a8"
            ai_move.from_piece = "R"
            ai_move.to_square = "d1" if board.turn == chess.WHITE else "d8"
            ai_move.to_piece = ""
            move_pub.publish(ai_move) # then rook move
            return
    elif 'x' in move: # capture
        ai_move.move_type = "capture"
        from_position, to_position = move.split('x')
        ai_move.from_square = from_position[1:3] if len(from_position) > 2 else from_position
        ai_move.from_piece = from_position[0] if len(from_position) > 2 else "P"
        ai_move.to_square = to_position[0:2]
        check_piece = board.piece_at(chess.parse_square(ai_move.to_square))
        ai_move.to_piece = check_piece.symbol().lower() if check_piece else ""
        if '=' in move:
            ai_move.promotion_piece = move.split('=')[1]
    elif '=' in move: # promotion
        ai_move.move_type = "promotion"
        from_position, to_position = move.split('-')
        to_position, promotion = to_position.split('=')
        ai_move.from_square = from_position[1:3] if len(from_position) > 2 else from_position
        ai_move.from_piece = "P"
        ai_move.to_square = to_position
        ai_move.to_piece = ""
        ai_move.promotion_piece = promotion
    elif '-' in move: # normal move
        ai_move.move_type = "move"
        from_position, to_position = move.split('-')
        ai_move.from_square = from_position[1:3] if len(from_position) > 2 else from_position
        ai_move.from_piece = from_position[0] if len(from_position) > 2 else "p"
        ai_move.to_square = to_position[0:2]
        check_piece = board.piece_at(chess.parse_square(ai_move.to_square))
        ai_move.to_piece = check_piece.symbol().lower() if check_piece else ""
        
    ai_move.notation = move
    move_pub.publish(ai_move)



# === CHESS BOARD SETUP ===
board = chess.Board()
last_move = None
comp_turn = False
move_history = []
undo_history = []

print("[INFO] 'u'=undo, 'U'=undo 2 moves, 'q'=quit.")
show_board(board)

try:
    while not board.is_game_over():
        key = cv2.waitKey(1) & 0xFF # maybe?

        # === Undo 1 move ===
        if key == ord('u'):
            if move_history:
                mv = move_history.pop()
                board.pop()
                print(f"[UNDO] Removed last move: {mv}")
                show_board(board)
            else:
                print("[INFO] No moves to undo.")

        # === Undo 2 moves ===
        if key == ord('U'):
            if len(move_history) >= 2:
                mv2 = move_history.pop()
                mv1 = move_history.pop()
                board.pop()
                board.pop()
                print(f"[UNDO] Removed last 2 moves: {mv1}, {mv2}")
                show_board(board)
            else:
                print("[INFO] Not enough moves to undo 2 times.")

        if key == ord('r'):
            if undo_history:
                mv = undo_history.pop()
                board.push(mv)
                move_history.append(mv)
                print(f"[REDO] Reapplied move: {mv}")
                show_board(board)
            else:
                print("[INFO] No moves to redo.")
        
        if key == ord('q'):
            print("[INFO] Quitting.")
            break

        # === COMPUTER TURN ===
        if comp_turn:
            result = engine.play(board, chess.engine.Limit(time=random.uniform(0.4, 0.9)))
            mv = result.move
            lan_move = board.lan(mv)
            print(f"LAN MOVE: {lan_move}")
            publish_robot_move(move = lan_move, board=board)
            board.push(mv)
            move_history.append(mv)
            last_move = mv
            print(f"[AI] Computer played: {mv.uci()}")
            show_board(board, last_move)
            comp_turn = False
        else:
            player_move = input("Enter your move (e.g. e2e4): ")
            from_sq = player_move[0:2]
            to_sq = player_move[2:]
            
            prev_board = board.copy()

            piece_a = prev_board.piece_at(chess.parse_square(from_sq))
            piece_b = prev_board.piece_at(chess.parse_square(to_sq[:2])) # in case of promotion, to_sq might have 3 chars like e8q, so we only want the square part
            
            if from_sq and to_sq:
                move = from_sq + to_sq
                try:
                    mv = chess.Move.from_uci(move)
                    lan_move = board.lan(mv)
                    if mv in board.legal_moves:
                        publish_robot_move(move = lan_move, board=board)
                        board.push(mv)
                        move_history.append(mv)
                        last_move = mv
                        print(f"[YOU] You played: {move}")
                        show_board(board, last_move)
                    else:
                        print(f"[!] Invalid move: {move}")
                except Exception as e:
                    print(f"[!] Error interpreting move: {e}")

    print("[INFO] Game over.")
finally:
    cv2.destroyAllWindows()
    engine.quit()