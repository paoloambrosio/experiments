scalaVersion := "2.11.5"

scalacOptions := Seq("-unchecked", "-deprecation", "-encoding", "utf8")

enablePlugins(GatlingPlugin)

libraryDependencies ++= {
  val gatlingV = "2.1.4"
  Seq(
    "io.gatling.highcharts" % "gatling-charts-highcharts" % gatlingV % "test",
    "io.gatling"            % "gatling-test-framework"    % gatlingV % "test"
  )
}

