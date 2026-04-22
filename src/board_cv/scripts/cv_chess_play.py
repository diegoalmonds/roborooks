#!/usr/bin/env python3

import pyrealsense2 as rs
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
import threading
import queue
import rospy

from board_cv.msg import Move

rospy.init_node("board_vision")
move_pub = rospy.Publisher("/ai_move", Move, queue_size=10)

# === CONFIG ===
CALIB_JSON = "sqdict.json"
ENGINE_PATH = r"/usr/games/stockfish"
MOVE_THRESHOLD = 25
MIN_CONTOUR_AREA = 250
CAM_INDEX = 2
BOARD_ORIENTATION = "TOP"  # "TOP", "BOTTOM", "SIDE_L", "SIDE_R"

DEBUG_MODE = False  # Press 'd' to toggle ON/OFF

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

# =============================================================================
# === STDIN THREAD (Stream Deck input) ========================================
# =============================================================================
# The Stream Deck writes characters to terminal stdin (e.g. "f\n", "e\n", "2\n").
# cv2.waitKey() only catches keypresses when the OpenCV window is focused, so
# it never sees Stream Deck input. This background thread reads stdin
# continuously and puts lines into a queue the main loop can poll safely.

stdin_queue = queue.Queue()

def stdin_reader():
    """Background thread: reads stdin lines and enqueues them."""
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
    """Non-blocking poll of the stdin queue. Returns None if nothing available."""
    try:
        return stdin_queue.get_nowait()
    except queue.Empty:
        return None

# =============================================================================
# === MANUAL OVERRIDE STATE MACHINE ===========================================
# =============================================================================
# Tracks what stage of the manual move entry we are in so the main loop
# knows what to do with the next stdin token.
#
# States:
#   idle          - normal CV mode, waiting for 'f' to trigger override
#   await_type    - waiting for move type: 'r' regular, 'c' castle, 'p' promotion
#   await_from_f  - waiting for from-square file letter (a-h)
#   await_from_r  - waiting for from-square rank (1-8)
#   await_to_f    - waiting for to-square file letter (a-h)
#   await_to_r    - waiting for to-square rank (1-8)
#   await_promo   - waiting for promotion piece (q, r, b, n)
#   await_castle  - waiting for castle side: 'k' kingside, 'q' queenside

override_state = "idle"
override_buf = {}   # accumulates from_f, from_r, to_f, to_r, promo

def reset_override():
    global override_state, override_buf
    override_state = "idle"
    override_buf = {}
    print("[OVERRIDE] Cancelled — back to CV mode.")

def override_prompt():
    """Print what the user/Stream Deck should send next."""
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
    """
    Feed one stdin token into the override state machine.
    Returns a chess.Move if a complete move has been assembled, else None.
    'x' at any point cancels the override and returns to CV mode.
    """
    global override_state, override_buf

    token = token.strip().lower()

    if token == 'x':
        reset_override()
        return None

    # --- await_type ---
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

    # --- await_castle ---
    if override_state == "await_castle":
        is_white = board.turn == chess.WHITE
        if token == 'k':   # kingside
            uci = "e1g1" if is_white else "e8g8"
        elif token == 'q': # queenside
            uci = "e1c1" if is_white else "e8c8"
        else:
            print(f"[OVERRIDE] Unknown castle side '{token}'. Use k/q.")
            override_prompt()
            return None
        try:
            mv = chess.Move.from_uci(uci)
            if mv in board.legal_moves:
                override_state = "idle"
                override_buf = {}
                return mv
            else:
                print(f"[OVERRIDE] Castle move {uci} is not legal right now.")
                reset_override()
                return None
        except Exception as e:
            print(f"[OVERRIDE] Castle error: {e}")
            reset_override()
            return None

    # --- file/rank collection ---
    if override_state == "await_from_f":
        if token in 'abcdefgh' and len(token) == 1:
            override_buf['from_f'] = token
            override_state = "await_from_r"
        else:
            print(f"[OVERRIDE] Invalid file '{token}'. Use a-h.")
        override_prompt()
        return None

    if override_state == "await_from_r":
        if token in '12345678' and len(token) == 1:
            override_buf['from_r'] = token
            override_state = "await_to_f"
        else:
            print(f"[OVERRIDE] Invalid rank '{token}'. Use 1-8.")
        override_prompt()
        return None

    if override_state == "await_to_f":
        if token in 'abcdefgh' and len(token) == 1:
            override_buf['to_f'] = token
            override_state = "await_to_r"
        else:
            print(f"[OVERRIDE] Invalid file '{token}'. Use a-h.")
        override_prompt()
        return None

    if override_state == "await_to_r":
        if token in '12345678' and len(token) == 1:
            override_buf['to_r'] = token
            if override_buf.get('promotion'):
                override_state = "await_promo"
                override_prompt()
                return None
            else:
                # assemble move
                uci = override_buf['from_f'] + override_buf['from_r'] + \
                      override_buf['to_f']   + override_buf['to_r']
                return _finalise_uci(uci, board)
        else:
            print(f"[OVERRIDE] Invalid rank '{token}'. Use 1-8.")
        override_prompt()
        return None

    if override_state == "await_promo":
        if token in ('q', 'r', 'b', 'n') and len(token) == 1:
            uci = override_buf['from_f'] + override_buf['from_r'] + \
                  override_buf['to_f']   + override_buf['to_r'] + token
            return _finalise_uci(uci, board)
        else:
            print(f"[OVERRIDE] Invalid promotion piece '{token}'. Use q/r/b/n.")
        override_prompt()
        return None

    return None

def _finalise_uci(uci, board):
    """Validate UCI string against legal moves and return Move or None."""
    global override_state, override_buf
    try:
        mv = chess.Move.from_uci(uci)
        if mv in board.legal_moves:
            override_state = "idle"
            override_buf = {}
            print(f"[OVERRIDE] Move accepted: {uci}")
            return mv
        else:
            print(f"[OVERRIDE] Move {uci} is not legal. Override cancelled.")
            reset_override()
            return None
    except Exception as e:
        print(f"[OVERRIDE] Invalid move '{uci}': {e}. Override cancelled.")
        reset_override()
        return None

# =============================================================================
# === HELPERS =================================================================
# =============================================================================

def poly_center(pts):
    a = np.array(pts, np.int32)
    M = cv2.moments(a)
    if M["m00"] == 0:
        return int(a[:, 0].mean()), int(a[:, 1].mean())
    return int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])

def find_square(x, y):
    """Return square name if point (x,y) is inside the polygon for that square."""
    pt = (float(x), float(y))
    for sq, pts in sq_points.items():
        poly = np.array(pts, np.int32)
        if cv2.pointPolygonTest(poly, pt, False) >= 0:
            return sq
    return None

def overlay_poly(frame, poly_pts, color, alpha=0.45):
    overlay = frame.copy()
    pts = np.array(poly_pts, np.int32)
    cv2.fillPoly(overlay, [pts], color)
    return cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0)

def draw_board_labels(base_frame):
    overlay = base_frame.copy()
    font = cv2.FONT_HERSHEY_SIMPLEX
    for sq, pts in sq_points.items():
        p = np.array(pts, np.int32)
        cv2.polylines(overlay, [p], True, (255, 255, 255), 1)
        if sq == "a1":
            cx, cy = poly_center(pts)
            mapped = remap_square(sq)
            cv2.putText(overlay, mapped, (cx - 12, cy + 5), font, 0.45, (0, 255, 255), 1, cv2.LINE_AA)
    return overlay

def show_board(board, last_move=None):
    svg = chess.svg.board(board=board, lastmove=last_move, coordinates=True, size=450)
    png_data = cairosvg.svg2png(bytestring=svg.encode('utf-8'))
    img = Image.open(io.BytesIO(png_data))
    img_cv = cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)
    cv2.imshow("Board State", img_cv)
    cv2.waitKey(1)

def draw_contours_debug(frame, contours):
    dbg = frame.copy()
    for i, c in enumerate(contours):
        area = cv2.contourArea(c)
        x, y, w, h = cv2.boundingRect(c)
        M = cv2.moments(c)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
        else:
            cx, cy = x + w // 2, y + h // 2
        cv2.rectangle(dbg, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.circle(dbg, (cx, cy), 3, (0, 0, 255), -1)
        cv2.putText(dbg, f"A:{int(area)}", (x, y - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    return dbg

# =============================================================================
# === PUBLISH MOVE TO ROBOT ===================================================
# =============================================================================

def publish_move(move_lan, board):
    """
    Publish a LAN-notation move to the /ai_move topic so chess_waypoint.py
    can execute the physical robot movement.
    Works for both the AI's moves and manually overridden player moves.
    """
    ai_move = Move()
    ai_move.is_checkmate = board.is_checkmate()

    if 'O' in move_lan:  # castle
        ai_move.move_type = "castle"
        is_white = board.turn == chess.WHITE
        if move_lan == "O-O":  # kingside
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
        else:  # queenside O-O-O
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

    elif 'x' in move_lan:  # capture
        ai_move.move_type = "capture"
        from_part, to_part = move_lan.split('x')
        ai_move.from_square = from_part[1:3] if len(from_part) > 2 else from_part
        ai_move.from_piece = from_part[0] if len(from_part) > 2 else "P"
        ai_move.to_square = to_part[0:2]
        check_piece = board.piece_at(chess.parse_square(ai_move.to_square))
        ai_move.to_piece = check_piece.symbol().upper() if check_piece else ""
        if '=' in move_lan:
            ai_move.promotion_piece = move_lan.split('=')[1][0]

    elif '=' in move_lan:  # promotion (non-capture)
        ai_move.move_type = "promotion"
        from_part, rest = move_lan.split('-')
        to_part, promo = rest.split('=')
        ai_move.from_square = from_part[1:3] if len(from_part) > 2 else from_part
        ai_move.from_piece = "P"
        ai_move.to_square = to_part[0:2]
        ai_move.to_piece = ""
        ai_move.promotion_piece = promo[0]

    elif '-' in move_lan:  # normal move
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
# === CAMERA ==================================================================
# =============================================================================

pipe = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)

try:
    profile = pipe.start(config)
except Exception as e:
    print("❌ RealSense could not be opened:", e)
    exit()

# =============================================================================
# === GAME STATE ==============================================================
# =============================================================================

# board = chess.Board("rnbqkbnr/1P6/8/8/8/8/8/RNBQKBNR w KQkq - 0 1") testing promotion
board = chess.Board()
ref_frame = None
last_move = None
comp_turn = False # if True, robot plays white, else robot plays black
move_history = []

print("[INFO] Stream Deck / keyboard commands:")
print("  f  = manual move override  |  u = undo  |  U = undo 2  |  d = debug  |  q = quit")
print("  During override:  x = cancel override at any time")
show_board(board)

# =============================================================================
# === MAIN LOOP ===============================================================
# =============================================================================

try:
    while not board.is_game_over():
        frames = pipe.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        rgb = np.asanyarray(color_frame.get_data())
        frame_raw = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

        display = draw_board_labels(frame_raw.copy())

        # Show override indicator on camera window when in override mode
        if override_state != "idle":
            cv2.putText(display, f"OVERRIDE: {override_state}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        cv2.imshow("Chess Tracker", display)
        key = cv2.waitKey(1) & 0xFF

        # =====================================================================
        # === STDIN / STREAM DECK INPUT ========================================
        # =====================================================================
        # Poll the stdin queue each frame. This is where Stream Deck input lands.
        token = get_stdin_line()

        # Handle top-level commands (work in any state)
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
            token = None  # consumed

        if token == 'U' or key == ord('U'):
            if override_state != "idle":
                reset_override()
            elif len(move_history) >= 2:
                mv2 = move_history.pop()
                mv1 = move_history.pop()
                board.pop()
                board.pop()
                print(f"[UNDO] Removed last 2 moves: {mv1}, {mv2}")
                show_board(board)
            else:
                print("[INFO] Not enough moves to undo 2 times.")
            token = None

        # Toggle debug (keyboard only — no Stream Deck button for this)
        if key == ord('d'):
            DEBUG_MODE = not DEBUG_MODE
            state = "ON" if DEBUG_MODE else "OFF"
            print(f"[INFO] Debug mode: {state}")
            if not DEBUG_MODE:
                for win in ("Diff", "Contours"):
                    try:
                        if cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) >= 0:
                            cv2.destroyWindow(win)
                    except cv2.error:
                        pass

        # =====================================================================
        # === OVERRIDE STATE MACHINE ==========================================
        # =====================================================================

        # 'f' from Stream Deck or keyboard triggers override entry
        if (token == 'f' or key == ord('f')) and override_state == "idle":
            override_state = "await_type"
            ref_frame = None  # discard any pending CV ref frame
            print("[OVERRIDE] Manual override activated. CV detection paused.")
            override_prompt()
            token = None

        # Feed token into state machine if we are inside an override
        if token and override_state != "idle":
            completed_move = handle_override_token(token, board)
            if completed_move is not None:
                # Push the move and publish to robot
                lan = board.lan(completed_move)
                publish_move(lan, board)
                board.push(completed_move)
                move_history.append(completed_move)
                last_move = completed_move
                print(f"[OVERRIDE] Played: {completed_move.uci()} ({lan})")
                show_board(board, last_move)

                # Highlight override move on camera view (blue from, purple to)
                try:
                    uci = completed_move.uci()
                    fh = overlay_poly(frame_raw.copy(), sq_points[uci[:2]], (255, 100, 0), 0.5)
                    fh = overlay_poly(fh, sq_points[uci[2:4]], (255, 0, 180), 0.5)
                    fh = draw_board_labels(fh)
                    cv2.imshow("Chess Tracker", fh)
                    cv2.waitKey(700)
                except Exception:
                    pass

                comp_turn = True

        # =====================================================================
        # === CV DETECTION (only when not in override mode) ===================
        # =====================================================================

        if override_state == "idle" and (key == ord('r') or token == 'r'):
            if ref_frame is None:
                ref_frame = frame_raw.copy()
                print("[DEBUG] Initial frame saved.")
            else:
                print("[DEBUG] Final frame captured, processing...")

                g1 = 0.5 * ref_frame[:, :, 2] + 0.4 * ref_frame[:, :, 1] + 0.1 * ref_frame[:, :, 0]
                g2 = 0.5 * frame_raw[:, :, 2]  + 0.4 * frame_raw[:, :, 1]  + 0.1 * frame_raw[:, :, 0]
                g1 = cv2.GaussianBlur(g1.astype(np.uint8), (5, 5), 0)
                g2 = cv2.GaussianBlur(g2.astype(np.uint8), (5, 5), 0)
                diff = cv2.absdiff(g1, g2)
                diff = cv2.GaussianBlur(diff, (3, 3), 0)
                diff = cv2.convertScaleAbs(diff, alpha=1.3, beta=0)
                _, diff_thresh = cv2.threshold(diff, MOVE_THRESHOLD, 255, cv2.THRESH_BINARY)

                diff_m = cv2.dilate(diff_thresh, None, iterations=4)
                diff_m = cv2.erode(diff_m, None, iterations=2)

                mask_board = np.zeros_like(diff_m)
                for pts in sq_points.values():
                    cv2.fillPoly(mask_board, [np.array(pts, np.int32)], 255)
                diff_m = cv2.bitwise_and(diff_m, mask_board)

                if DEBUG_MODE:
                    cv2.imshow("Diff", diff_m)

                kernel = np.ones((3, 3), np.uint8)
                diff_m = cv2.morphologyEx(diff_m, cv2.MORPH_OPEN, kernel)
                diff_m = cv2.morphologyEx(diff_m, cv2.MORPH_CLOSE, kernel)

                contours, _ = cv2.findContours(diff_m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                contours_filtered = [c for c in contours if cv2.contourArea(c) > MIN_CONTOUR_AREA]

                def candidates_for_contour(c):
                    x, y, w, h = cv2.boundingRect(c)
                    M = cv2.moments(c)
                    cx_m = int(M["m10"] / M["m00"]) if M["m00"] != 0 else x + w // 2
                    y_factors = [0.20, 0.30, 0.40]
                    x_jitters = [0, -6, 6]
                    cands = []
                    for yf in y_factors:
                        cy_try = int(y + yf * h)
                        for xj in x_jitters:
                            sq_try = find_square(cx_m + xj, cy_try)
                            if sq_try:
                                cands.append((sq_try, cx_m + xj, cy_try))
                    seen = set(); out = []
                    for s in cands:
                        if s[0] not in seen:
                            seen.add(s[0]); out.append(s)
                    return out

                contour_cand_lists = [(c, candidates_for_contour(c)) for c in contours_filtered]
                detected = set()
                chosen_mapping = []
                prev_board = board.copy()

                if len(contour_cand_lists) >= 2:
                    contour_cand_lists = sorted(contour_cand_lists,
                                                key=lambda it: cv2.contourArea(it[0]),
                                                reverse=True)[:2]
                    (c0, list0), (c1, list1) = contour_cand_lists
                    found = False
                    for s0, cx0, cy0 in list0:
                        for s1, cx1, cy1 in list1:
                            for fr, to in [(s0, s1), (s1, s0)]:
                                try:
                                    mv = chess.Move.from_uci(fr + to)
                                except Exception:
                                    continue
                                if mv in prev_board.legal_moves:
                                    detected = {fr, to}
                                    chosen_mapping = [
                                        (fr, cx0, cy0) if fr == s0 else (fr, cx1, cy1),
                                        (to, cx1, cy1) if to == s1 else (to, cx0, cy0)
                                    ]
                                    found = True
                                    break
                            if found: break
                        if found: break
                    if not found and list0 and list1:
                        s0, s1 = list0[0][0], list1[0][0]
                        detected = {s0, s1}
                        chosen_mapping = [(s0, list0[0][1], list0[0][2]),
                                          (s1, list1[0][1], list1[0][2])]

                elif len(contour_cand_lists) == 1:
                    c0, list0 = contour_cand_lists[0]
                    if list0:
                        dest_chosen = None
                        for sq_try, cx_try, cy_try in list0:
                            if any(m.uci()[2:] == sq_try for m in prev_board.legal_moves):
                                dest_chosen = (sq_try, cx_try, cy_try)
                                break
                        chosen = dest_chosen or list0[0]
                        detected = {chosen[0]}
                        chosen_mapping = [chosen]

                if DEBUG_MODE:
                    print(f"[DEBUG] Detected squares: {detected}")
                    if chosen_mapping:
                        dbg = frame_raw.copy()
                        for sq, cx, cy in chosen_mapping:
                            poly = np.array(sq_points[sq], np.int32)
                            cv2.polylines(dbg, [poly], True, (0, 255, 0), 2)
                            cv2.circle(dbg, (int(cx), int(cy)), 4, (0, 0, 255), -1)
                            cv2.putText(dbg, sq, (int(cx) + 6, int(cy)),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                        cv2.imshow("Contours", dbg)

                # === interpret move ===
                from_sq, to_sq = None, None
                if len(detected) == 2:
                    a, b = list(detected)
                    piece_a = prev_board.piece_at(chess.parse_square(a))
                    piece_b = prev_board.piece_at(chess.parse_square(b))
                    if piece_a and not piece_b:
                        from_sq, to_sq = a, b
                    elif piece_b and not piece_a:
                        from_sq, to_sq = b, a
                    else:
                        def rank_idx(s): return int(s[1])
                        if board.turn == chess.WHITE:
                            from_sq, to_sq = sorted([a, b], key=rank_idx)
                        else:
                            from_sq, to_sq = sorted([a, b], key=rank_idx, reverse=True)

                elif len(detected) == 1:
                    to_sq = list(detected)[0]
                    piece_now = board.piece_at(chess.parse_square(to_sq))
                    if piece_now:
                        candidates = [m for m in board.legal_moves if m.uci()[2:] == to_sq]
                        chosen = None
                        for m in candidates:
                            if prev_board.piece_at(chess.parse_square(m.uci()[:2])):
                                chosen = m; break
                        if not chosen and candidates:
                            chosen = candidates[0]
                        if chosen:
                            from_sq = chosen.uci()[:2]
                            to_sq = chosen.uci()[2:]
                    else:
                        file = to_sq[0]; rank = int(to_sq[1]); fi = files.index(file)
                        search_offsets = ([(0,-1),(-1,0),(1,0),(-1,-1),(1,-1),(0,-2)]
                                          if board.turn == chess.WHITE
                                          else [(0,1),(-1,0),(1,0),(-1,1),(1,1),(0,2)])
                        search_offsets += [(-1,1),(1,1),(-1,-1),(1,-1)]
                        found = False
                        for df, dr in search_offsets:
                            f_idx = fi + df; r_idx = rank + dr
                            if 0 <= f_idx < 8 and 1 <= r_idx <= 8:
                                adj = f"{files[f_idx]}{r_idx}"
                                if prev_board.piece_at(chess.parse_square(adj)):
                                    try_mv = chess.Move.from_uci(adj + to_sq)
                                    if try_mv in board.legal_moves:
                                        from_sq = adj; found = True; break
                        if not found:
                            for df in (-1,0,1):
                                for dr in (-1,0,1):
                                    if df == 0 and dr == 0: continue
                                    f_idx = fi+df; r_idx = rank+dr
                                    if 0 <= f_idx < 8 and 1 <= r_idx <= 8:
                                        adj = f"{files[f_idx]}{r_idx}"
                                        if prev_board.piece_at(chess.parse_square(adj)):
                                            try:
                                                try_mv = chess.Move.from_uci(adj+to_sq)
                                            except Exception:
                                                try_mv = None
                                            if try_mv and try_mv in board.legal_moves:
                                                from_sq = adj; found = True; break
                                            if not from_sq:
                                                from_sq = adj
                                if found: break

                # === execute CV-detected move ===
                if from_sq and to_sq:
                    move_uci = from_sq + to_sq
                    try:
                        mv = chess.Move.from_uci(move_uci)
                        if mv in board.legal_moves:
                            lan = board.lan(mv)
                            # publish_move(lan, board)   # <-- was missing in original
                            board.push(mv)
                            move_history.append(mv)
                            last_move = mv
                            print(f"[YOU] You played: {move_uci} ({lan})")
                            show_board(board, last_move)

                            try:
                                fh = overlay_poly(frame_raw.copy(), sq_points[from_sq], (0, 255, 0), 0.5)
                                fh = overlay_poly(fh, sq_points[to_sq], (0, 0, 255), 0.5)
                                fh = draw_board_labels(fh)
                                cv2.imshow("Chess Tracker", fh)
                                cv2.waitKey(700)
                            except Exception:
                                pass

                            comp_turn = True
                        else:
                            print(f"[!] CV detected invalid move: {move_uci}")
                    except Exception as e:
                        print(f"[!] Error interpreting CV move: {e}")

                ref_frame = None

        # =====================================================================
        # === COMPUTER TURN ===================================================
        # =====================================================================

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

            try:
                uci = mv.uci()
                fai = overlay_poly(frame_raw.copy(), sq_points[uci[:2]], (0, 255, 255), 0.45)
                fai = overlay_poly(fai, sq_points[uci[2:]], (0, 165, 255), 0.45)
                fai = draw_board_labels(fai)
                cv2.imshow("Chess Tracker", fai)
                cv2.waitKey(900)
            except Exception:
                pass

            comp_turn = False

    print("[INFO] Game over.")

finally:
    try:
        pipe.stop()
    except Exception:
        pass
    cv2.destroyAllWindows()
    engine.quit()