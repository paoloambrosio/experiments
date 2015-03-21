FROM dockerfile/java:oracle-java8
MAINTAINER Paolo Ambrosio <dockerfile@paolo.ambrosio.name>

RUN mkdir -p /opt/upstream
COPY build/libs/upstream-spring-*.jar /opt/upstream/upstream.jar
WORKDIR /opt/upstream

EXPOSE 8080
ENTRYPOINT java \
  -Dgraphite.host=$GRAPHITE_PORT_2003_TCP_ADDR -Dgraphite.port=$GRAPHITE_PORT_2003_TCP_PORT \
  -Dservice.downstream.url=http://$DOWNSTREAM_PORT_9000_TCP_ADDR:$DOWNSTREAM_PORT_9000_TCP_PORT/ \
  -jar upstream.jar
