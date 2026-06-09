"""
Autonomous PiCar navigator driven by a Claude vision agent.

The agent runs a perceive → decide → act loop:
  1. Capture a JPEG snapshot from the car's camera
  2. Send the image to Claude with the navigation context
  3. Claude calls one navigation tool (move_forward / move_backward / turn / arrived)
  4. Execute the command on the car
  5. Repeat until the exit is found or max_steps is reached

Usage:
    python navigator.py
    python navigator.py --max-steps 30

Requires ANTHROPIC_API_KEY to be set in the environment.
"""

import argparse
import sys

import anthropic

import picar_control as car

client = anthropic.Anthropic()

SYSTEM_PROMPT = """\
You are the navigation brain of a PiCar robot. Your sole objective is to \
navigate the car around a room until you find the exit (a door, open doorway, \
or clear passage leading out of the room).

After each action you will be shown a fresh camera image from the car's \
forward-facing perspective. Based on what you see, issue exactly one \
navigation command.

Navigation guidelines:
- Move forward when there is clear open space ahead.
- Turn when you are blocked, facing a wall, or need to explore a new direction.
- Prefer cautious steps: 0.3–0.8 m forward, 20–45° turns.
- The car uses Ackermann (car-style) steering — it cannot rotate on the spot; \
  turns produce an arc while moving forward.
- Keep a mental map of directions already explored to avoid circling.
- The exit is likely a door or open gap in the walls.  \
  When you can see it clearly ahead or have passed through it, use 'arrived'.
"""

TOOLS = [
    {
        "name": "move_forward",
        "description": (
            "Drive the PiCar forward. Use when there is clear space ahead."
        ),
        "input_schema": {
            "type": "object",
            "properties": {
                "metres": {
                    "type": "number",
                    "description": "Distance to travel in metres (0.1 – 2.0).",
                },
                "reasoning": {
                    "type": "string",
                    "description": "Brief description of what you see and why you chose this command.",
                },
            },
            "required": ["metres", "reasoning"],
        },
    },
    {
        "name": "move_backward",
        "description": (
            "Reverse the PiCar. Use to escape a dead-end or create turning room."
        ),
        "input_schema": {
            "type": "object",
            "properties": {
                "metres": {
                    "type": "number",
                    "description": "Distance to reverse in metres (0.1 – 1.0).",
                },
                "reasoning": {"type": "string"},
            },
            "required": ["metres", "reasoning"],
        },
    },
    {
        "name": "turn",
        "description": (
            "Steer the PiCar through an arc while moving forward. "
            "Positive degrees = right, negative = left."
        ),
        "input_schema": {
            "type": "object",
            "properties": {
                "degrees": {
                    "type": "number",
                    "description": (
                        "Heading change in degrees. "
                        "Positive = right, negative = left. "
                        "Typical range: –90 to 90."
                    ),
                },
                "reasoning": {"type": "string"},
            },
            "required": ["degrees", "reasoning"],
        },
    },
    {
        "name": "arrived",
        "description": (
            "Signal that the exit has been found and reached. "
            "Use only when you can clearly see the exit ahead or have passed through it."
        ),
        "input_schema": {
            "type": "object",
            "properties": {
                "reasoning": {
                    "type": "string",
                    "description": "Describe the exit you found.",
                }
            },
            "required": ["reasoning"],
        },
    },
]


def _image_block(b64: str) -> dict:
    return {
        "type": "image",
        "source": {"type": "base64", "media_type": "image/jpeg", "data": b64},
    }


def _execute(name: str, tool_input: dict) -> str:
    """Execute a navigation tool and return a short result string."""
    reasoning = tool_input.get("reasoning", "")
    if name == "move_forward":
        metres = float(tool_input["metres"])
        print(f"  → FORWARD {metres:.2f} m  |  {reasoning}")
        car.move_forward(metres)
        return f"Moved forward {metres} m."
    if name == "move_backward":
        metres = float(tool_input["metres"])
        print(f"  ← BACKWARD {metres:.2f} m  |  {reasoning}")
        car.move_backward(metres)
        return f"Reversed {metres} m."
    if name == "turn":
        degrees = float(tool_input["degrees"])
        direction = "right" if degrees > 0 else "left"
        print(f"  ↺ TURN {abs(degrees):.0f}° {direction}  |  {reasoning}")
        car.turn(degrees)
        return f"Turned {degrees}°."
    if name == "arrived":
        print(f"  ✓ ARRIVED  |  {reasoning}")
        return "arrived"
    return f"Unknown command: {name}"


def navigate(max_steps: int = 50) -> bool:
    """Run the navigation loop.

    Returns True if the exit was found, False if max_steps was exhausted.
    """
    print(f"PaiCar navigator starting (max {max_steps} steps).\n")

    messages: list[dict] = []
    last_tool_use_id: str | None = None
    last_result: str | None = None

    for step in range(1, max_steps + 1):
        print(f"[Step {step}/{max_steps}] Capturing snapshot…")
        image_b64 = car.snapshot()

        # Build the next user turn
        if last_tool_use_id is None:
            # Very first turn — no previous tool call to close
            user_content = [
                _image_block(image_b64),
                {"type": "text", "text": "Initial view. Issue your first navigation command."},
            ]
        else:
            # Close the previous tool call, then attach the new image
            user_content = [
                {
                    "type": "tool_result",
                    "tool_use_id": last_tool_use_id,
                    "content": last_result,
                },
                _image_block(image_b64),
                {"type": "text", "text": f"Step {step}: new view after last action."},
            ]

        messages.append({"role": "user", "content": user_content})

        response = client.messages.create(
            model="claude-opus-4-8",
            max_tokens=1024,
            system=SYSTEM_PROMPT,
            tools=TOOLS,
            tool_choice={"type": "any"},
            messages=messages,
        )

        messages.append({"role": "assistant", "content": response.content})

        tool_use = next((b for b in response.content if b.type == "tool_use"), None)
        if tool_use is None:
            print("No tool call in response — stopping.")
            break

        last_tool_use_id = tool_use.id
        last_result = _execute(tool_use.name, tool_use.input)

        if tool_use.name == "arrived":
            print("\nNavigation complete — exit reached!")
            return True

    print(f"\nNavigation ended after {step} step(s) without finding the exit.")
    return False


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="PaiCar autonomous navigator")
    parser.add_argument(
        "--max-steps",
        type=int,
        default=50,
        help="Maximum navigation steps before giving up (default: 50)",
    )
    args = parser.parse_args()

    found = navigate(max_steps=args.max_steps)
    sys.exit(0 if found else 1)
