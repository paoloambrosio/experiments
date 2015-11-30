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
  .aggregate(projectToRef(client))

lazy val client = (project in file("client"))
  .enablePlugins(ScalaJSPlugin, ScalaJSPlay)
  .settings(
    scalaVersion := scalaV,
    persistLauncher in Compile := true
  )
  .dependsOn(d3cloud)

lazy val d3cloud = (project in file("scala-js-d3-cloud"))
  .enablePlugins(ScalaJSPlugin)
  .settings(
    resolvers += Resolver.mavenLocal,
    scalaVersion := scalaV,
    libraryDependencies += "org.singlespaced" %%% "scalajs-d3" % "0.1.1",
    jsDependencies += "org.webjars" % "d3-cloud" % "1.2.1" / "d3.layout.cloud.js"
  )
