enablePlugins(AshScriptPlugin)

name := "kafka-streams"
version := "1.0"

libraryDependencies ++= Seq(
  "org.apache.kafka" % "kafka-streams" % "1.0.0",
  "org.apache.kafka" % "kafka-clients" % "1.0.0",
  "org.slf4j" % "slf4j-simple" % "1.7.25"
//  "com.uber.jaeger" % "jaeger-core" % "0.24.0",
//  "io.opentracing.contrib" % "opentracing-kafka-streams" % "0.0.8"
)

dockerBaseImage := "openjdk:8-jre"
dockerUpdateLatest := true
