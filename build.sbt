organization := "net.paoloambrosio.exp-akka-cluster"
version      := "0.1"
scalaVersion := "2.11.7"

val akkaV = "2.4.1"

libraryDependencies ++= Seq(
  "com.typesafe.akka" %% "akka-cluster" % akkaV,
  "com.typesafe.akka" %% "akka-cluster-tools" % akkaV,
  "com.typesafe.akka" %% "akka-cluster-metrics" % akkaV
)
