#!/bin/bash

docker rmi my-grafana
docker build --rm=true -t my-grafana .
