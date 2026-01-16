def test_import():
    import proto_aegis_grpc  # noqa: F401
    import proto_aegis_grpc.v1  # noqa: F401


def test_import_v1_messages():
    from proto_aegis_grpc.v1 import robot_srvs_pb2  # noqa: F401
    from proto_aegis_grpc.v1 import robot_srvs_pb2_grpc  # noqa: F401
    from proto_aegis_grpc.v1 import geometry_msgs_pb2  # noqa: F401
    from proto_aegis_grpc.v1 import control_msgs_pb2  # noqa: F401
    from proto_aegis_grpc.v1 import sensor_msgs_pb2  # noqa: F401
