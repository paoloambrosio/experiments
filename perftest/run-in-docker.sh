#!/bin/bash

export JAVA_OPTS="-Dgatling.data.graphite.host=$GRAPHITE_PORT_2003_TCP_ADDR -Dgatling.data.graphite.port=$GRAPHITE_PORT_2003_TCP_PORT -Dundertest.host=$UNDERTEST_PORT_8080_TCP_ADDR -Dundertest.port=$UNDERTEST_PORT_8080_TCP_PORT"

gatling.sh -s gatling.SystemSimulation
