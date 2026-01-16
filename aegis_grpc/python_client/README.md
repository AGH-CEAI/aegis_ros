# aegis_grpc_client

A gRPC client to read and control the Aegis robot.

## Build notes
Before building, make sure that the protobuf messages (`proto_aegis_grpc` [package](../python_proto/README.md)) are generated (see [this README](../README.md)).

## Run tests
```bash
poetry lock
poetry install
poetry run pytest -v -s
```
