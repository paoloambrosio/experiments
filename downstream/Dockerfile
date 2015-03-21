FROM dockerfile/java:oracle-java8
MAINTAINER Paolo Ambrosio <dockerfile@paolo.ambrosio.name>

COPY target/universal/stage /opt/downstream
WORKDIR /opt/downstream

EXPOSE 9000
ENTRYPOINT bin/downstream \
  -Dgraphite.host=$GRAPHITE_PORT_2003_TCP_ADDR -Dgraphite.port=$GRAPHITE_PORT_2003_TCP_PORT
