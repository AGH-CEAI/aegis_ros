def test_init_AegisRobotClient():
    from aegis_grpc_client import AegisRobotClient

    client_default = AegisRobotClient()  # noqa: F841
    client_custom = AegisRobotClient("127.0.0.1:50051")  # noqa: F841
