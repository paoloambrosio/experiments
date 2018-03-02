enablePlugins(AshScriptPlugin)

name := "finagle-thrift"
version := "1.0"

libraryDependencies ++= Seq(
  "org.slf4j" % "slf4j-simple" % "1.7.25",
  "io.zipkin.finagle" %% "zipkin-finagle-http" % "1.2.2" % Runtime,
  "com.google.guava" % "guava" % "24.0-jre" % Runtime // For some reason it's not a dependency, removed after 1.2.2
)

dockerBaseImage := "openjdk:8-jre-alpine"
dockerUpdateLatest := true
