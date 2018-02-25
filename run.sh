#!/bin/sh

sbt ';compile ;docker:publishLocal'

cd docker-compose; docker-compose up; cd -

