#!/bin/bash

docker run -d -p 80:80 --expose=2003 --name grafana kamon/grafana_graphite
docker run -d --name downstream --link grafana:graphite my-downstream
docker run -d --name upstream --link grafana:graphite --link downstream:downstream my-upstream-spring

echo "### Don't forget to upload the dashboard into Grafana! ###"

