FROM dockerfile/java:oracle-java8
MAINTAINER Paolo Ambrosio <dockerfile@paolo.ambrosio.name>

RUN mkdir -p /opt/sysintsim
COPY build/libs/service-*-standalone.jar /opt/sysintsim/service-akka.jar
WORKDIR /opt/sysintsim

EXPOSE 9000
ENTRYPOINT java \
  -Dgraphite.host=$GRAPHITE_PORT_2003_TCP_ADDR -Dgraphite.port=$GRAPHITE_PORT_2003_TCP_PORT \
  -Dapplication.downstream.host=$DOWNSTREAM_PORT_9000_TCP_ADDR \
  -Dapplication.downstream.port=$DOWNSTREAM_PORT_9000_TCP_PORT \
  -jar service-akka.jar
