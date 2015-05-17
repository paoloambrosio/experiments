Dependencies:
 - Gradle to build sources
 - Docker and Docker-Compose to run simulation on Linux containers

How to run in Docker:
 $ gradle build
 $ ./docker-sboot-akka-up # or docker-akka-sboot-up for Akka upstream
 - Point the Web browser to http://localhost/ to see the dashboard
 $ ./docker-run-perftest

Use CTRL+C to tear down running services.

