How to run:
 $ docker run --rm -p 80:80 -p 2003:2003 --name kamon-grafana-dashboard kamon/grafana_graphite
 - Point browser to http://localhost/ and import the dashboard
 $ cd downstream; sbt run
 $ cd upstream-spring; gradle run
 $ cd perftest; sbt 'testOnly gatling.UpstreamSimulation'

To change the downstream slowdown distribution:
 $ sbt -Dapplication.slowdown-strategy.distribution=constant2s run



Collect and graph stats from Gatling and Downstream:
 - Gatling conf file to point to Graphite
 - JmxTrans conf to point to Graphite
 - Run Grafana Docker image exposing Grafana and Graphite ports
   docker run -d -p 80:80 -p 2003:2003 --name kamon-grafana-dashboard kamon/grafana_graphite
   (https://github.com/kamon-io/docker-grafana-graphite)
 - Run app (sbt run) and performance test (sbt test)
 - Add metrics to dashboard

Exercise Spring Boot Upstream:
 - As before, run downstream app and Graphite
 - Run upstream app (gradle run)
 - Run performance test (sbt 'testOnly gatling.UpstreamSimulation')
 - Watch the synchronous app die ;-)

