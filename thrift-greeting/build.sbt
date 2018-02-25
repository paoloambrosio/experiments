name := "thrift-greeting"
version := "1.0"

libraryDependencies ++= Seq(
  "org.apache.thrift" % "libthrift" % "0.9.3",
  "com.twitter" %% "scrooge-core" % "18.2.0" exclude("com.twitter", "libthrift"),
  "com.twitter" %% "finagle-thrift" % "18.2.0" exclude("com.twitter", "libthrift")
)
