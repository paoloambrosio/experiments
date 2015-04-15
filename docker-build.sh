#!/bin/bash

function build {
  cd $1; ./docker-build.sh; cd -
}

build service-spring-boot
build service-akka
build perftest
build grafana
