enablePlugins(AshScriptPlugin)

name := "finagle-thrift"
version := "1.0"

libraryDependencies ++= Seq(
  "org.apache.thrift" % "libthrift" % "0.9.3",
  "com.twitter" %% "scrooge-core" % "18.2.0" exclude("com.twitter", "libthrift"),
  "com.twitter" %% "finagle-thrift" % "18.2.0" exclude("com.twitter", "libthrift"),
  "org.slf4j" % "slf4j-simple" % "1.7.25",
  "com.uber.jaeger" % "jaeger-core" % "0.24.0",
  "io.opentracing.contrib" % "opentracing-finagle" % "0.0.2"
)

dockerBaseImage := "openjdk:8-jre-alpine"
dockerUpdateLatest := true
