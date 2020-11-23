#!/bin/bash
docker build --tag=brittany:prueba .
docker run -it brittany:prueba /bin/bash
