#!/bin/bash

sbt compile
docker rmi my-perftest
docker build --rm=true -t my-perftest .
