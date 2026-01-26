#!/bin/bash

cmake -S . -B build
cmake --build build --target generate_protos

pip uninstall proto_aegis_grpc aegis_grpc_client -y
cd python_proto
poetry build
pip install ./dist/proto_aegis_grpc-*.whl
cd ../python_client
poetry build
pip install ./dist/aegis_grpc_client-*.whl
