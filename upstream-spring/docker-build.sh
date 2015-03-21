#!/bin/bash

gradle build
docker rmi my-upstream-spring
docker build --rm=true -t my-upstream-spring .
