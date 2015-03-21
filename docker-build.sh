#!/bin/bash

function build {
  cd $1; ./docker-build.sh; cd -
}

build downstream
build upstream-spring
build perftest

