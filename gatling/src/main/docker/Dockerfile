FROM denvazh/gatling
MAINTAINER Paolo Ambrosio <dockerfile@paolo.ambrosio.name>

COPY gatling.conf $GATLING_HOME/conf/
COPY gatling $GATLING_HOME/user-files/simulations/gatling
COPY runSystemSimulation.sh $GATLING_HOME/bin/

ENTRYPOINT ["runSystemSimulation.sh"]
