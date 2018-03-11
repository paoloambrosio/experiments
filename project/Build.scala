import sbt.Keys._
import sbt._


object BuildSettings {
  val buildSettings = Seq(
    organization := "net.paoloambrosio.exp-akka-clbst",
    version      := "0.1",
    scalaVersion := "2.12.4"
  )
}

object Dependencies {
  private val akkaV = "2.5.11"
  private val akkaHttpV = "10.1.0"

  val akka = Seq(
    "com.typesafe.akka" %% "akka-actor"   % akkaV,
    "com.typesafe.akka" %% "akka-remote"  % akkaV,
    "com.typesafe.akka" %% "akka-cluster" % akkaV
  )
  val akkaHttp = Seq(
    "com.typesafe.akka" %% "akka-http"         % akkaHttpV,
    "de.heikoseeberger" %% "akka-http-jackson" % "1.20.0"
  )
}

object Resolvers {
  val akkaHttpJsonResolvers = Resolver.bintrayRepo("hseeberger", "maven")
}
