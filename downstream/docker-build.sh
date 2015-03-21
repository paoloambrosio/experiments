#!/bin/bash

sbt stage
docker rmi my-downstream
docker build --rm=true -t my-downstream .
