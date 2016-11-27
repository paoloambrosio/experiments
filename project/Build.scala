import sbt.Keys._
import sbt._


object BuildSettings {
  val buildSettings = Seq(
    organization := "net.paoloambrosio.exp-akka-loctr",
    version      := "0.1",
    scalaVersion := "2.11.8"
  )
}

object Dependencies {
  private val akkaV = "2.4.12"
  private val akkaHttpV = "2.4.11"

  val akka = Seq(
    "com.typesafe.akka" %% "akka-actor"             % akkaV,
    "com.typesafe.akka" %% "akka-remote"            % akkaV,
    "com.typesafe.akka" %% "akka-cluster"           % akkaV
  )
  val akkaHttp = Seq(
    "com.typesafe.akka" %% "akka-http-experimental" % akkaHttpV,
    "de.heikoseeberger" %% "akka-http-jackson"      % "1.10.1"
  )
}

object Resolvers {
  val akkaHttpJsonResolvers = Resolver.bintrayRepo("hseeberger", "maven")
}
