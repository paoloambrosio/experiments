#!/bin/sh

mvn -f jmx-webapp/pom.xml package && mvn -f agent/pom.xml package
java -javaagent:agent/target/agent-0.1-SNAPSHOT.jar -jar jmx-webapp/target/jmx-webapp-0.1-SNAPSHOT.jar

