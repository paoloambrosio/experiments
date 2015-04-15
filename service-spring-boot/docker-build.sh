#!/bin/bash

gradle build
docker rmi my-service-spring-boot
docker build --rm=true -t my-service-spring-boot .
