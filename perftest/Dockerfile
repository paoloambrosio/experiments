FROM denvazh/gatling
MAINTAINER Paolo Ambrosio <dockerfile@paolo.ambrosio.name>

COPY src/test/resources/gatling.conf $GATLING_HOME/conf/
COPY src/test/scala $GATLING_HOME/user-files/simulations/
COPY run-in-docker.sh $GATLING_HOME/bin/

ENTRYPOINT run-in-docker.sh