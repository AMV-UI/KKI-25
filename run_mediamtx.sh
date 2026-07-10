#!/bin/bash
docker run --rm -it --network=host -v "$PWD/mediamtx.yml:/mediamtx.yml:ro" bluenviron/mediamtx:1
