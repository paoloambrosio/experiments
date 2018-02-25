lazy val `finagle-http` = project.dependsOn(`thrift-greeting`)
lazy val `finagle-thrift` = project.dependsOn(`thrift-greeting`)
lazy val `thrift-greeting` = project

lazy val `spring-boot` = project
