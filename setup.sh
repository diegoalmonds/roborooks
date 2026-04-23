#!/bin/bash

# RoboRooks Launch Script
# Opens two terminals: one for cv_chess_play and one for chess_waypoint.py

ROBOROOKS_DIR="$HOME/roborooks"
SOURCE_FILE="$ROBOROOKS_DIR/sawyer_wheels.sh"

# Detect available terminal emulator
if command -v gnome-terminal &> /dev/null; then
    TERM_CMD="gnome-terminal"
elif command -v xterm &> /dev/null; then
    TERM_CMD="xterm"
elif command -v konsole &> /dev/null; then
    TERM_CMD="konsole"
else
    echo "No supported terminal emulator found (gnome-terminal, xterm, konsole)."
    exit 1
fi

echo "Launching RoboRooks..."

# Terminal 1: CV node
if [ "$TERM_CMD" = "gnome-terminal" ]; then
    gnome-terminal --title="CV Chess Play" -- bash -c "
        cd $ROBOROOKS_DIR && source $SOURCE_FILE && rosrun board_cv cv_chess_play;
        exec bash"
elif [ "$TERM_CMD" = "xterm" ]; then
    xterm -title "CV Chess Play" -e "cd $ROBOROOKS_DIR && source $SOURCE_FILE && rosrun board_cv cv_chess_play; exec bash" &
elif [ "$TERM_CMD" = "konsole" ]; then
    konsole --title "CV Chess Play" -e bash -c "cd $ROBOROOKS_DIR && source $SOURCE_FILE && rosrun board_cv cv_chess_play; exec bash" &
fi

# Small delay to avoid race conditions on ROS init
sleep 2

# Terminal 2: Chess waypoint node
if [ "$TERM_CMD" = "gnome-terminal" ]; then
    gnome-terminal --title="Chess Waypoint" -- bash -c "
        cd $ROBOROOKS_DIR && source $SOURCE_FILE && rosrun chess_play chess_waypoint.py;
        exec bash"
elif [ "$TERM_CMD" = "xterm" ]; then
    xterm -title "Chess Waypoint" -e "cd $ROBOROOKS_DIR && source $SOURCE_FILE && rosrun chess_play chess_waypoint.py; exec bash" &
elif [ "$TERM_CMD" = "konsole" ]; then
    konsole --title "Chess Waypoint" -e bash -c "cd $ROBOROOKS_DIR && source $SOURCE_FILE && rosrun chess_play chess_waypoint.py; exec bash" &
fi

echo "Both nodes launched."