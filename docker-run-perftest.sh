#!/bin/bash

docker run --rm --link grafana:graphite --link upstream:undertest my-perftest

