FROM dockerfile/java:oracle-java8
MAINTAINER Paolo Ambrosio <dockerfile@paolo.ambrosio.name>

RUN mkdir -p /opt/sysintsim
COPY build/libs/service-*.jar /opt/sysintsim/service-spring-boot.jar
WORKDIR /opt/sysintsim

EXPOSE 8080
ENTRYPOINT java \
  -Dgraphite.host=$GRAPHITE_PORT_2003_TCP_ADDR -Dgraphite.port=$GRAPHITE_PORT_2003_TCP_PORT \
  -Dservice.downstream.url=http://$DOWNSTREAM_PORT_9000_TCP_ADDR:$DOWNSTREAM_PORT_9000_TCP_PORT/ \
  -Dconfig.enable.circuit-breaker=$CONFIG_ENABLE_CIRCUIT_BREAKER \
  -jar service-spring-boot.jar
