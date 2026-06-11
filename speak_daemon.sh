#!/bin/bash
# TTS daemon — watches /tmp/speak_queue.txt and speaks new lines as they arrive.
# Start once before a long run: ./speak_daemon.sh &
# Send text: echo "some message" >> /tmp/speak_queue.txt

QUEUE=/tmp/speak_queue.txt
SPEED=${1:-145}
VOICE=${2:-en}

touch "$QUEUE"
echo "[speak_daemon] started, watching $QUEUE" >&2

# hw:0,0 = Jabra USB speaker (card 0). Without -d, espeak-ng defaults to HDMI.
DEVICE=${3:-hw:0,0}

tail -n 0 -f "$QUEUE" | while IFS= read -r line; do
    [[ -z "$line" ]] && continue
    espeak-ng -s "$SPEED" -v "$VOICE" -d "$DEVICE" -- "$line" 2>/dev/null
done
