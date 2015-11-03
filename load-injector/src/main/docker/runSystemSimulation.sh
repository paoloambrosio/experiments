#!/bin/bash

export JAVA_OPTS="
    -Dundertest.host=$UNDERTEST_PORT_8080_TCP_ADDR
    -Dundertest.port=$UNDERTEST_PORT_8080_TCP_PORT
  "

gatling.sh -s gatling.SystemSimulation
