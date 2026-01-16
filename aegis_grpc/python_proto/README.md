# proto_aegis_grpc

Package with auto-generated gRPC protobuf messages for the Aegis project.

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
import_test.test_import()
import_test.test_import_v1_messages()
```
