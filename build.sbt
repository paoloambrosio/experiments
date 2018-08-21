import Dependencies._

enablePlugins(AkkaGrpcPlugin)
enablePlugins(JavaAgent)

lazy val root = (project in file(".")).
  settings(
    inThisBuild(List(
      organization := "com.example",
      scalaVersion := "2.12.6",
      version      := "0.1.0-SNAPSHOT"
    )),
    name := "akka-grpc-example",
    libraryDependencies += scalaTest % Test,
    javaAgents += "org.mortbay.jetty.alpn" % "jetty-alpn-agent" % "2.0.7" % "runtime"
)
