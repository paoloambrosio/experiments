package gatling

import io.gatling.core.Predef._
import io.gatling.http.Predef._
import scala.concurrent.duration._

class DownstreamSimulation extends Simulation {

  val httpConf = http.baseURL("http://localhost:9000")

  val scn = scenario("DownstreamSimulation")
    .forever { exec(http("OnlyRequest").get("/")) }

  setUp(
    scn
      .inject(atOnceUsers(2000))
      .throttle(
        reachRps(500) in (30 seconds),
        holdFor(90 seconds)
      )
  ).protocols(httpConf)

}
