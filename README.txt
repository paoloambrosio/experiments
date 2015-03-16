

Collect and graph stats from Gatling and App:
 - Gatling conf file to point to Graphite
 - JmxTrans conf to point to Graphite
 - Run Grafana Docker image exposing Grafana and Graphite ports
   docker run -d -p 80:80 -p 2003:2003 --name kamon-grafana-dashboard kamon/grafana_graphite
   (https://github.com/kamon-io/docker-grafana-graphite)
 - Run app (sbt run) and performance test (sbt test)
 - Add metrics to dashboard

