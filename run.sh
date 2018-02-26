#!/bin/sh

sbt ';compile ;docker:publishLocal'

cd docker-compose; docker-compose up; cd -

# docker exec -it dockercompose_kafka_1 /usr/bin/kafka-console-producer --topic lines --broker-list kafka:9092
# docker exec -it dockercompose_kafka_1 /usr/bin/kafka-console-consumer  --topic lineslength --bootstrap-server kafka:9092
