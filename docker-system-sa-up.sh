#!/bin/bash

docker run -d -p 80:80 --expose=2003 --name grafana my-grafana
docker run -d --name downstream --link grafana:graphite my-service-akka
docker run -d --name upstream --link grafana:graphite --link downstream:downstream -e CONFIG_ENABLE_CIRCUIT_BREAKER=false my-service-spring-boot
