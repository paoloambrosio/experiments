enablePlugins(AshScriptPlugin)

name := "spring-boot"
version := "1.0"

val springBootVersion = "1.5.10.RELEASE"

libraryDependencies ++= Seq(
  "org.springframework.boot" % "spring-boot-starter-parent" % springBootVersion,
  "org.springframework.boot" % "spring-boot-starter-web" % springBootVersion)

dockerBaseImage := "openjdk:8-jre-alpine"
dockerUpdateLatest := true
