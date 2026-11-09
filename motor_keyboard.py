"""Hold-to-run keyboard controller for the Blue Pill motor firmware."""

import argparse
import sys
import threading

import serial
from pynput import keyboard


MOVEMENT_KEYS = {"w", "a", "s", "d"}


def main() -> int:
    parser = argparse.ArgumentParser(description="Control the motor board from the keyboard.")
    parser.add_argument("--port", required=True, help="Serial port, for example COM5")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate (default: 115200)")
    args = parser.parse_args()

    try:
        connection = serial.Serial(args.port, args.baud, timeout=0.05, write_timeout=0.5)
    except serial.SerialException as exc:
        print(f"Could not open {args.port}: {exc}", file=sys.stderr)
        return 1

    write_lock = threading.Lock()
    held_keys: list[str] = []

    def send(command: str) -> bool:
        try:
            with write_lock:
                connection.write(command.encode("ascii"))
            return True
        except serial.SerialException as exc:
            print(f"\nSerial write failed: {exc}", file=sys.stderr)
            return False

    def key_character(key: keyboard.Key | keyboard.KeyCode) -> str | None:
        character = getattr(key, "char", None)
        return character.lower() if character else None

    def on_press(key: keyboard.Key | keyboard.KeyCode) -> bool | None:
        if key == keyboard.Key.esc:
            return False

        character = key_character(key)
        if character in MOVEMENT_KEYS:
            if character not in held_keys:
                held_keys.append(character)
                if not send(character):
                    return False
        elif character == "x" or key == keyboard.Key.space:
            held_keys.clear()
            if not send("x"):
                return False
        elif character == "f":
            held_keys.clear()
            if not send("f"):
                return False
        elif character == "r":
            if not send("r"):
                return False
        return None

    def on_release(key: keyboard.Key | keyboard.KeyCode) -> bool | None:
        character = key_character(key)
        if character in held_keys:
            was_active = character == held_keys[-1]
            held_keys.remove(character)
            if was_active:
                command = held_keys[-1] if held_keys else "x"
                if not send(command):
                    return False
        return None

    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener_started = False

    try:
        send("x")
        print(f"Controlling {args.port} at {args.baud} baud.")
        print("Hold W/A/S/D to move; release to stop or resume another held key.")
        print("F moves one cell; Space/X stops, R resets counts, Esc exits.")
        listener.start()
        listener_started = True

        while listener.is_alive():
            line = connection.readline()
            if line:
                print(line.decode("ascii", errors="replace").rstrip())
    except KeyboardInterrupt:
        print("\nStopping motors...")
    except serial.SerialException as exc:
        print(f"\nSerial connection failed: {exc}", file=sys.stderr)
        return 1
    finally:
        if listener_started:
            listener.stop()
            try:
                listener.join()
            finally:
                send("x")
                connection.close()
        else:
            send("x")
            connection.close()

    print("Motors stopped; controller closed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())