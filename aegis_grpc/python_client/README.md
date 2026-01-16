# aegis_grpc_client

A gRPC client to read and control the Aegis robot.

## Build notes
Before building, make sure that the protobuf messages (`proto_aegis_grpc` [package](../python_proto/README.md)) are generated (see [this README](../README.md)).

## Run tests

### poetry
```bash
poetry lock
poetry install
poetry run pytest -v -s
```

### Manual
```bash
cd ./test
python3
# in python3 console
import import_test
import init_test
import_test.test_import()
import_test.test_import_AegisRobotClient()
init_test.test_init_AegisRobotClient()
```
