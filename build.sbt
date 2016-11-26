import BuildSettings._
import Dependencies._
import Resolvers._
import com.typesafe.sbt.packager.archetypes.JavaAppPackaging.autoImport.bashScriptExtraDefines
import com.typesafe.sbt.packager.docker._

lazy val commonSettings = buildSettings ++ Seq(
  resolvers += akkaHttpJsonResolvers
)

lazy val dockerSettings = Seq(
  dockerExposedPorts := Seq(Docker.akkaTcpPort),
  bashScriptExtraDefines := Seq(Docker.bashExports)
)

lazy val frontendDockerSettings = dockerSettings ++ Seq(
  dockerCommands := Seq(
    Cmd("FROM", "java:latest"),
    Cmd("USER", "root"),
    ExecCmd("RUN", "apt-get", "-qq", "update"),
    ExecCmd("RUN", "apt-get", "-yq", "install", "dnsutils"),
    ExecCmd("RUN", "apt-get", "clean"),
    ExecCmd("RUN", "rm", "-rf", "/var/lib/apt/lists/*")
  ) ++ dockerCommands.value.filterNot {
    case Cmd("FROM", _) => true
    case _ => false
  },
  bashScriptExtraDefines := Seq(Docker.frontendBashExports)
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
  .settings(frontendDockerSettings)
  .dependsOn(`backend-api`, common)

lazy val backend = project
  .settings(commonSettings: _*)
  .settings(libraryDependencies ++= akka)
  .enablePlugins(JavaAppPackaging)
  .enablePlugins(DockerPlugin)
  .settings(dockerSettings)
  .dependsOn(`backend-api`, common)

lazy val `backend-api` = project
  .settings(commonSettings: _*)
  .settings(libraryDependencies ++= akka)
