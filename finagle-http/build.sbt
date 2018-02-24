enablePlugins(AshScriptPlugin)

name := "finagle-http"
version := "1.0"

libraryDependencies ++= Seq(
  "com.twitter" %% "finagle-http" % "18.2.0",
  "org.slf4j" % "slf4j-simple" % "1.7.25",
  "com.uber.jaeger" % "jaeger-core" % "0.24.0",
  "io.opentracing.contrib" % "opentracing-finagle" % "0.0.2"
)

dockerBaseImage := "openjdk:8-jre-alpine"
dockerUpdateLatest := true
