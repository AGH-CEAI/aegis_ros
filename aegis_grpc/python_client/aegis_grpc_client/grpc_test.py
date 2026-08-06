import grpc

# Assuming your AegisWledClient class and the pb2 imports are defined above this
from aegis_grpc_client.grpc_client import AegisWledClient

# import wled_service_pb2
# import wled_service_pb2_grpc


def run_wled_client():
    # 1. Define the server target address and port
    server_target = "localhost:50051"
    print(f"Connecting to WLED gRPC server at {server_target}...")

    # 2. Open an insecure channel (use secure_channel if your server requires TLS/SSL)
    with grpc.insecure_channel(server_target) as channel:
        # Instantiate your custom client wrapper
        client = AegisWledClient(channel)

        try:
            # --- Example 1: Define a new scene ---
            # print("\n[1] Defining a new scene...")
            # success, message = client.define_scene(
            #     scene_name="Neon_Nights",
            #     color="#FF00FF",
            #     brightness=200
            # )
            # print(f"Success: {success}, Message: '{message}'")

            # --- Example 2: Change the active scene ---
            print("\n[2] Changing scene...")
            success, message = client.change_scene(
                scene="scene_2", section="section_4", effect_id=0
            )
            print(f"Success: {success}, Message: '{message}'")

            # # --- Example 3: Retrieve all scenes ---
            # print("\n[3] Fetching available scenes...")
            # scene_names, brightnesses = client.get_scenes()
            # for name, bright in zip(scene_names, brightnesses):
            #     print(f" - Scene: {name} (Brightness: {bright})")

            # # --- Example 4: Retrieve LED sections ---
            # print("\n[4] Fetching sections...")
            # section_names, starts, stops = client.get_sections()
            # for name, start, stop in zip(section_names, starts, stops):
            #     print(f" - Section: {name} [LEDs {start} to {stop}]")

            # # --- Example 5: Stream effects (Server-side Streaming) ---
            # print("\n[5] Streaming effects (listening for 3 events)...")
            # stream = client.stream_effects()

            # Since stream_effects returns a generator, we can iterate over it
            # events_received = 0
            # for effect_data in stream:
            #     print(f"Received effect update: {effect_data}")
            #     events_received += 1
            #     if events_received >= 3:
            #         print("Received 3 effect updates, stopping stream.")
            #         break # Break early just for the sake of the example

        except grpc.RpcError as e:
            # Catch gRPC-specific errors (e.g., server down, method unimplemented)
            print("\ngRPC Error Encountered:")
            print(f"Status Code: {e.code()}")
            print(f"Details: {e.details()}")


if __name__ == "__main__":
    run_wled_client()
