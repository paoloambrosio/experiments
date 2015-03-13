name         := "downstream"
organization := "net.paoloambrosio.sysintsim"
version      := "0.1"
scalaVersion := "2.11.5"

scalacOptions := Seq("-unchecked", "-deprecation", "-encoding", "utf8")

libraryDependencies ++= {
  val akkaV       = "2.3.9"
  val akkaStreamV = "1.0-M2"
  Seq(
    "com.typesafe.akka"    %% "akka-actor"                        % akkaV,
    "com.typesafe.akka"    %% "akka-stream-experimental"          % akkaStreamV,
    "com.typesafe.akka"    %% "akka-http-core-experimental"       % akkaStreamV,
    "com.typesafe.akka"    %% "akka-http-experimental"            % akkaStreamV,
    "com.typesafe.akka"    %% "akka-http-spray-json-experimental" % akkaStreamV
  )
}

enablePlugins(GatlingPlugin)

libraryDependencies ++= Seq(
  "io.gatling.highcharts" % "gatling-charts-highcharts" % "2.1.4" % "test",
  "io.gatling"            % "gatling-test-framework"    % "2.1.4" % "test"
)