import argparse
import asyncio

import grpc
from aegis_grpc_client.grpc_client import AegisRobotClient

"""
AI generated code for testing grpc connection.
This script provides a command-line interface to interact with the WLED gRPC server,
allowing users to define scenes, change scenes, retrieve available scenes and sections, and effects.
Available command-line parameters:
change-scene --section section_1 --scene scene_2 --effect 0
define-scene --scene-name section_5 --color 255 0 255 --brightness 200
get-scenes
get-sections
get-effects
"""


async def define_scene_command(
    client: AegisRobotClient, scene_name: str, color: list[int], brightness: int
) -> None:
    print(f"\n[1] Defining new scene '{scene_name}' with color {color}...")
    try:
        success, msg = await client.wled_define_scene(
            scene_name=scene_name, color=color, brightness=brightness
        )
        print(f"Success: {success}, msg: '{msg}'")
    except grpc.RpcError as e:
        print(f"\ngRPC Error: {e.code()} - {e.details()}")


async def change_scene_command(
    client: AegisRobotClient, section: str, scene: str, effect_id: int
) -> None:
    print(f"\n[2] Changing scene to '{scene}' on section '{section}'...")
    try:
        success, msg = await client.wled_change_scene(
            scene=scene, section=section, effect_id=effect_id
        )
        print(f"Success: {success}, msg: '{msg}'")
    except grpc.RpcError as e:
        print(f"\ngRPC Error: {e.code()} - {e.details()}")


async def get_scenes_command(client: AegisRobotClient) -> None:
    print("\n[3] Fetching available scenes...")
    try:
        scene_names, brightnesses = await client.wled_get_scenes()
        for name, bright in zip(scene_names, brightnesses):
            print(f" - Scene: {name} (Brightness: {bright})")
    except grpc.RpcError as e:
        print(f"\ngRPC Error: {e.code()} - {e.details()}")


async def get_sections_command(client: AegisRobotClient) -> None:
    print("\n[4] Fetching sections...")
    try:
        section_names, starts, stops = await client.wled_get_sections()
        for name, start, stop in zip(section_names, starts, stops):
            print(f" - Section: {name} [LEDs {start} to {stop}]")
    except grpc.RpcError as e:
        print(f"\ngRPC Error: {e.code()} - {e.details()}")


async def get_effects_command(client: AegisRobotClient) -> None:
    print("\n[5] Fetching effects...")

    try:
        effects_dict = await client.wled_get_effects()
        print(f"Available {len(effects_dict)} effect(s):")
        for name, idx in effects_dict.items():
            print(f"  ID: {idx} \t| NAME: {name}")
    except grpc.RpcError as e:
        print(f"\ngRPC Error: {e.code()} - {e.details()}")


async def run(args) -> None:
    server_target = "127.0.0.1:50051"
    print(f"Connecting to WLED gRPC server at {server_target}...")

    async with AegisRobotClient(server_target).connect_context() as client:
        if args.command == "define-scene":
            await define_scene_command(
                client, args.scene_name, args.color, args.brightness
            )
        elif args.command == "change-scene":
            await change_scene_command(client, args.section, args.scene, args.effect)
        elif args.command == "get-scenes":
            await get_scenes_command(client)
        elif args.command == "get-sections":
            await get_sections_command(client)
        elif args.command == "get-effects":
            await get_effects_command(client)


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

    args = parser.parse_args()
    asyncio.run(run(args))


if __name__ == "__main__":
    main()
