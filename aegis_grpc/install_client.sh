#!/bin/bash
# Run this script as root (designed for containers)

cd ./protogen
cmake -S . -B /tmp/build -Wno-dev
cmake --build /tmp/build --target generate_protos

uv pip uninstall --system proto_aegis_grpc aegis_grpc_client
uv pip install --system ../python_proto
uv pip install --system ../python_client
