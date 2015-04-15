FROM dockerfile/java:oracle-java8
MAINTAINER Paolo Ambrosio <dockerfile@paolo.ambrosio.name>

COPY target/universal/stage /opt/sysintsim
WORKDIR /opt/sysintsim

EXPOSE 9000
ENTRYPOINT bin/service-akka \
  -Dgraphite.host=$GRAPHITE_PORT_2003_TCP_ADDR -Dgraphite.port=$GRAPHITE_PORT_2003_TCP_PORT
