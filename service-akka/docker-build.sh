#!/bin/bash

sbt stage
docker rmi my-service-akka
docker build --rm=true -t my-service-akka .
