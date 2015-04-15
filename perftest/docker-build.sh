#!/bin/bash

docker rmi my-perftest
docker build --rm=true -t my-perftest .
