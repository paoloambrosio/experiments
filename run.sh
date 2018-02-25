#!/bin/sh

sbt ';project finagle-http; compile; docker:publishLocal; project spring-boot; compile; docker:publishLocal'

cd docker-compose; docker-compose up; cd -

