"""
Run a trained PPO agent controlling the Roomba in the simulator,
narrating each command via the speak daemon.

Usage:
    # 1. Start the speak daemon (once per session):
    #    nohup bash -c 'tail -n 0 -f /tmp/speak_queue.txt | while IFS= read -r line; do espeak-ng -s 145 -- "$line" 2>/dev/null; done' > /tmp/speak_daemon.log 2>&1 &

    # 2. Start the simulator (no humans, real-time speed):
    #    cargo run --release -p simulator -- 1

    # 3. Run the agent:
    python run_agent.py models/best/best_model.zip
    python run_agent.py models/best/best_model.zip --speed 5
"""

import argparse
from collections import deque
import sys
import time

from stable_baselines3 import PPO

from roomba_env import ACTIONS, RoombaEnv

SPEAK_QUEUE = "/tmp/speak_queue.txt"


def speak(msg: str) -> None:
    with open(SPEAK_QUEUE, "a") as f:
        f.write(msg + "\n")


def get_shielded_action(obs, predicted_action, action_history) -> tuple[int, str | None]:
    front_dist_cm = obs[0] * 500.0
    back_dist_cm = obs[4] * 500.0
    left_dist_cm = obs[2] * 500.0
    right_dist_cm = obs[6] * 500.0
    bumper_active = obs[12] > 0.5

    # 1. Stuck / Oscillation detection
    is_stuck = False
    stuck_reason = ""
    
    # Check for alternating turn loop (e.g., [2, 3, 2, 3] or [3, 2, 3, 2])
    if len(action_history) >= 4:
        last_four = list(action_history)[-4:]
        if last_four == [2, 3, 2, 3] or last_four == [3, 2, 3, 2]:
            is_stuck = True
            stuck_reason = "oscillation loop"
            
    # Check for spinning in place while front is close (e.g. 4 turns in a row)
    if not is_stuck and len(action_history) >= 4 and front_dist_cm < 40.0:
        last_four = list(action_history)[-4:]
        if all(a in [2, 3, 4] for a in last_four):
            is_stuck = True
            stuck_reason = "spinning in place"

    if is_stuck:
        if back_dist_cm > 20.0:
            action_history.clear()
            return 1, f"{stuck_reason.capitalize()} detected override: forcing backup (move -15) to escape"
        else:
            action_history.clear()
            return 4, f"{stuck_reason.capitalize()} detected override: forcing turn 90 to break cycle"

    # 2. Bumper Active Check
    if bumper_active:
        if predicted_action != 1:
            if back_dist_cm > 15.0:
                return 1, "Bumper active override: forcing backup (move -15)"
            else:
                if left_dist_cm >= right_dist_cm:
                    return 4, "Bumper active (rear blocked) override: forcing turn 90"
                else:
                    return 3, "Bumper active (rear blocked) override: forcing turn right (turn -45)"

    # 3. Proximity check
    if front_dist_cm < 40.0 and predicted_action == 0:
        if front_dist_cm < 25.0 and back_dist_cm > 20.0:
            return 1, f"Front obstacle extremely close ({front_dist_cm:.1f} cm) override: forcing backup (move -15)"
        else:
            if left_dist_cm >= right_dist_cm:
                return 2, f"Front obstacle too close ({front_dist_cm:.1f} cm) override: turning left (turn 45)"
            else:
                return 3, f"Front obstacle too close ({front_dist_cm:.1f} cm) override: turning right (turn -45)"

    return predicted_action, None


def run(model_path: str, speed: int = 1) -> None:
    speak("Starting Roomba navigation. Loading trained agent.")

    env = RoombaEnv()
    try:
        model = PPO.load(model_path)
    except Exception as e:
        speak(f"Failed to load model: {e}")
        env.close()
        sys.exit(1)

    obs, _ = env.reset()
    # Override speed for this run (1 = real-time demo, higher for faster replay)
    env._cmd(f"speed {speed}")
    speak(f"Simulation speed set to {speed} times real time.")

    step = 0
    done = False
    action_history = deque(maxlen=4)

    while not done:
        action, _ = model.predict(obs, deterministic=True)
        action = int(action)

        # Apply safety shield
        shielded_action, override_msg = get_shielded_action(obs, action, action_history)
        if override_msg:
            speak(override_msg)
            action = shielded_action

        action_history.append(action)

        _, narration = ACTIONS[action]

        dist_mm = obs[8] * 60_000.0
        door_state = "open" if obs[9] > 0.5 else "closed"
        speak(
            f"Step {step + 1}: {narration}. "
            f"Exit door is {door_state}. "
            f"Distance to exit: {dist_mm:.0f} millimetres."
        )

        obs, reward, terminated, truncated, _ = env.step(action)
        done = terminated or truncated
        step += 1

    if terminated:
        speak(f"Success! Roomba exited through Door 17 after {step} steps.")
    else:
        speak(f"Episode ended after {step} steps without reaching the exit.")

    env.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("model", help="Path to a trained model .zip file")
    parser.add_argument(
        "--speed", type=int, default=1,
        help="Simulation speed multiplier (1 = real-time, default: 1)"
    )
    args = parser.parse_args()
    run(args.model, speed=args.speed)
