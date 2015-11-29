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
      "com.vmunier" %% "play-scalajs-scripts" % "0.3.0",
      "com.typesafe.akka" %% "akka-contrib" % "2.3.13"
    )
  )
  .dependsOn(sharedJvm)
  .aggregate(projectToRef(client))

lazy val client = (project in file("client"))
  .enablePlugins(ScalaJSPlugin, ScalaJSPlay)
  .settings(
    scalaVersion := scalaV,
    persistLauncher in Compile := true
  )
  .dependsOn(d3cloud, sharedJs)

lazy val d3cloud = (project in file("scala-js-d3-cloud"))
  .enablePlugins(ScalaJSPlugin)
  .settings(
    scalaVersion := scalaV,
    libraryDependencies += "org.singlespaced" %%% "scalajs-d3" % "0.1.1",
    jsDependencies += "org.webjars" % "d3-cloud" % "1.2.1" / "d3.layout.cloud.js"
  )

lazy val shared = (crossProject.crossType(CrossType.Pure) in file("shared"))
  .settings(scalaVersion := scalaV)
  .jsConfigure(_ enablePlugins ScalaJSPlay)

// Why do we need these to be defined explicitly?
lazy val sharedJvm = shared.jvm
lazy val sharedJs = shared.js