#!/bin/sh
python3 -m grpc_tools.protoc -I./src/core_gcs/protos --python_out=./src/core_gcs/core_gcs --pyi_out=./src/core_gcs/core_gcs --grpc_python_out=./src/core_gcs/core_gcs ./src/core_gcs/protos/server.proto
