import BuildSettings._
import Dependencies._
import Resolvers._

lazy val commonSettings = buildSettings ++ Seq(
  resolvers += akkaHttpJsonResolvers
)

lazy val root = (project in file("."))
  .settings(buildSettings: _*)
  .aggregate(frontend)

lazy val frontend = project
  .settings(commonSettings: _*)
  .settings(libraryDependencies ++= akkaHttp)
  .enablePlugins(JavaAppPackaging)
  .enablePlugins(DockerPlugin)
