import BuildSettings._
import Dependencies._
import Resolvers._

lazy val commonSettings = buildSettings ++ Seq(
  resolvers += akkaHttpJsonResolvers
)

lazy val root = (project in file("."))
  .settings(buildSettings: _*)
  .aggregate(frontend, backend)

lazy val common = project
  .settings(commonSettings: _*)
  .settings(libraryDependencies ++= akka)

lazy val frontend = project
  .settings(commonSettings: _*)
  .settings(libraryDependencies ++= akkaHttp)
  .enablePlugins(JavaAppPackaging)
  .enablePlugins(DockerPlugin)
  .dependsOn(`backend-api`, common)

lazy val backend = project
  .settings(commonSettings: _*)
  .enablePlugins(JavaAppPackaging)
  .enablePlugins(DockerPlugin)
  .dependsOn(`backend-api`, common)

lazy val `backend-api` = project
  .settings(commonSettings: _*)
  .settings(libraryDependencies ++= akka.map(_ % Provided))
