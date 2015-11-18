import sbt.Project.projectToRef

lazy val scalaV = "2.11.7"

version := "1.0-SNAPSHOT"

lazy val server = (project in file("server"))
  .enablePlugins(PlayScala)
  .settings(
    scalaVersion := scalaV,
    scalaJSProjects := Seq(client),
    pipelineStages := Seq(scalaJSProd, gzip),
    routesGenerator := InjectedRoutesGenerator,
    libraryDependencies ++= Seq(
      jdbc,
      "com.vmunier" %% "play-scalajs-scripts" % "0.3.0"
    )
  )
  .aggregate(projectToRef(client))

lazy val client = (project in file("client"))
  .enablePlugins(ScalaJSPlugin, ScalaJSPlay)
  .settings(
    scalaVersion := scalaV,
    persistLauncher in Compile := true
  )