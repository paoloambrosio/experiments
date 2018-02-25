enablePlugins(AshScriptPlugin)

name := "finagle-thrift"
version := "1.0"

libraryDependencies ++= Seq(
  "org.slf4j" % "slf4j-simple" % "1.7.25",
  "com.uber.jaeger" % "jaeger-core" % "0.24.0",
  "io.opentracing.contrib" % "opentracing-finagle" % "0.0.2"
)

dockerBaseImage := "openjdk:8-jre-alpine"
dockerUpdateLatest := true
