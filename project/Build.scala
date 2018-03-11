import sbt.Keys._
import sbt._


object BuildSettings {
  val buildSettings = Seq(
    organization := "net.paoloambrosio.exp-akka-clstr",
    version      := "0.1",
    scalaVersion := "2.12.4"
  )
}

object Dependencies {
  private val akkaV = "2.5.11"
  private val akkaHttpV = "10.1.0"
  private val akkaManagementV = "0.10.0"

  val akka = Seq(
    "com.typesafe.akka"             %% "akka-actor"                        % akkaV,
    "com.typesafe.akka"             %% "akka-cluster"                      % akkaV,
    "com.lightbend.akka.management" %% "akka-management-cluster-bootstrap" % akkaManagementV,
    "com.lightbend.akka.discovery"  %% "akka-discovery-dns"                % akkaManagementV
  )
  val akkaHttp = Seq(
    "com.typesafe.akka" %% "akka-http"         % akkaHttpV,
    "de.heikoseeberger" %% "akka-http-jackson" % "1.20.0"
  )
}

object Resolvers {
  val akkaHttpJsonResolvers = Resolver.bintrayRepo("hseeberger", "maven")
}
