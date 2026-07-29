def test_import():
    import proto_aegis_grpc
    import proto_aegis_grpc.v1  # noqa: F401


def test_import_v1_messages():
    from proto_aegis_grpc.v1 import (
        control_msgs_pb2,  # noqa: F401
        geometry_msgs_pb2,  # noqa: F401
        robot_srvs_pb2,  # noqa: F401
        robot_srvs_pb2_grpc,  # noqa: F401
        sensor_msgs_pb2,  # noqa: F401
    )
