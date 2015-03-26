How to run:
 $ cd grafana; ./docker-build.sh; cd -; docker run --rm -p 80:80 -p 2003:2003 --name grafana my-grafana
 $ cd downstream; sbt run
 $ cd upstream-spring; gradle run
 $ cd perftest; sbt test

To change the downstream slowdown distribution:
 $ sbt -Dapplication.slowdown-strategy.distribution=constant2s run

To run Gatling on the downstream only:
 $ sbt -Dundertest.port=9000 test


How to run in Docker:
 $ docker-build.sh
 $ docker-system-up.sh
 $ docker-run-perftest.sh
 $ docker-system-down.sh # When done
