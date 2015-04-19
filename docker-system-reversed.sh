#!/bin/bash

docker run -d -p 80:80 --expose=2003 --name grafana my-grafana
docker run -d -p 8080:80 --name downstream --link grafana:graphite -e CONFIG_ENABLE_CIRCUIT_BREAKER=false my-service-spring-boot
docker run -d -p 9000:80 --name upstream --link grafana:graphite --link downstream:downstream my-service-akka

