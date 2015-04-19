FROM dockerfile/java:oracle-java8
MAINTAINER Paolo Ambrosio <dockerfile@paolo.ambrosio.name>

RUN mkdir -p /opt/sysintsim
COPY build/libs/service-*.jar /opt/sysintsim/service-spring-boot.jar
WORKDIR /opt/sysintsim

EXPOSE 80
ENTRYPOINT java \
  -Dgraphite.host=$GRAPHITE_PORT_2003_TCP_ADDR \
  -Dgraphite.port=$GRAPHITE_PORT_2003_TCP_PORT \
  -Dservice.downstream.host=$DOWNSTREAM_PORT_80_TCP_ADDR \
  -Dservice.downstream.port=$DOWNSTREAM_PORT_80_TCP_PORT \
  -Dserver.port=80 \
#  -Dconfig.enable-circuit-breaker=$CONFIG_ENABLE_CIRCUIT_BREAKER \
  -jar service-spring-boot.jar
