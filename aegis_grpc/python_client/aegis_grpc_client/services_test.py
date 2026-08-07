import argparse

import grpc

from aegis_grpc_client.grpc_client import AegisWledClient

"""
AI generated code for testing grpc connection.
This script provides a command-line interface to interact with the WLED gRPC server,
allowing users to define scenes, change scenes, retrieve available scenes and sections, and stream effects.
Available command-line parameters:
change-scene --section section_1 --scene scene_2 --effect 0
define-scene --scene-name section_5 --color 255 0 255 --brightness 200
get-scenes
get-sections
stream-effects
"""


def define_scene_command(
    client, scene_name: str, color: list[int], brightness: int
) -> None:
    print(f"\n[1] Defining new scene '{scene_name}' with color {color}...")
    try:
        success, msg = client.define_scene(
            scene_name=scene_name, color=color, brightness=brightness
        )
        print(f"Success: {success}, msg: '{msg}'")
    except grpc.RpcError as e:
        print(f"\ngRPC Error: {e.code()} - {e.details()}")


def change_scene_command(client, section: str, scene: str, effect_id: int) -> None:
    print(f"\n[2] Changing scene to '{scene}' on section '{section}'...")
    try:
        success, msg = client.change_scene(
            scene=scene, section=section, effect_id=effect_id
        )
        print(f"Success: {success}, msg: '{msg}'")
    except grpc.RpcError as e:
        print(f"\ngRPC Error: {e.code()} - {e.details()}")


def get_scenes_command(client) -> None:
    print("\n[3] Fetching available scenes...")
    try:
        scene_names, brightnesses = client.get_scenes()
        for name, bright in zip(scene_names, brightnesses):
            print(f" - Scene: {name} (Brightness: {bright})")
    except grpc.RpcError as e:
        print(f"\ngRPC Error: {e.code()} - {e.details()}")


def get_sections_command(client) -> None:
    print("\n[4] Fetching sections...")
    try:
        section_names, starts, stops = client.get_sections()
        for name, start, stop in zip(section_names, starts, stops):
            print(f" - Section: {name} [LEDs {start} to {stop}]")
    except grpc.RpcError as e:
        print(f"\ngRPC Error: {e.code()} - {e.details()}")


def stream_effects_command(client, limit: int) -> None:
    print(f"\n[5] Streaming effects (listening for up to {limit} events)...")
    try:
        stream = client.stream_effects()
        for events_received, effect_data in enumerate(stream, start=1):
            print(f"Received effect update: {effect_data}")
            if events_received >= limit:
                print(f"Received {limit} effect updates, stopping stream.")
                break
    except grpc.RpcError as e:
        print(f"\ngRPC Error: {e.code()} - {e.details()}")


def main():
    parser = argparse.ArgumentParser(
        description="Complete WLED gRPC Client Management Tool"
    )
    subparsers = parser.add_subparsers(
        dest="command", required=True, help="Operation to perform"
    )

    # Subcommand: define-scene
    define_parser = subparsers.add_parser("define-scene", help="Define a new scene")
    define_parser.add_argument(
        "-n",
        "--scene-name",
        required=True,
        help="Name of the new scene (e.g., Neon_Nights)",
    )
    define_parser.add_argument(
        "-co",
        "--color",
        type=int,
        nargs="+",
        required=True,
        help="Color values as a list of integers (e.g., 255 0 255)",
    )
    define_parser.add_argument(
        "-b",
        "--brightness",
        type=int,
        required=True,
        help="Brightness level (0-255)",
    )

    # Subcommand: change-scene
    change_parser = subparsers.add_parser(
        "change-scene", help="Change the active scene and effect"
    )
    change_parser.add_argument(
        "-s", "--section", required=True, help="Specify the section (e.g., section_1)"
    )
    change_parser.add_argument(
        "-c", "--scene", required=True, help="Specify the scene (e.g., scene_2)"
    )
    change_parser.add_argument(
        "-e",
        "--effect",
        type=int,
        required=True,
        help="Specify the effect ID as an integer (e.g., 0)",
    )

    # Subcommand: get-scenes
    subparsers.add_parser("get-scenes", help="Retrieve all available scenes")

    # Subcommand: get-sections
    subparsers.add_parser("get-sections", help="Retrieve all LED sections")

    # Subcommand: stream-effects
    stream_parser = subparsers.add_parser(
        "stream-effects", help="Stream effects from the server"
    )
    stream_parser.add_argument(
        "-l",
        "--limit",
        type=int,
        default=3,
        help="Number of events to receive before stopping (default: 3)",
    )

    args = parser.parse_args()

    server_target = "localhost:50051"
    print(f"Connecting to WLED gRPC server at {server_target}...")

    with grpc.insecure_channel(server_target) as channel:
        client = AegisWledClient(channel)

        if args.command == "define-scene":
            define_scene_command(client, args.scene_name, args.color, args.brightness)
        elif args.command == "change-scene":
            change_scene_command(client, args.section, args.scene, args.effect)
        elif args.command == "get-scenes":
            get_scenes_command(client)
        elif args.command == "get-sections":
            get_sections_command(client)
        elif args.command == "stream-effects":
            stream_effects_command(client, args.limit)


if __name__ == "__main__":
    main()
