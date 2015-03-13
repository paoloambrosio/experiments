package gatling

import io.gatling.core.Predef._
import io.gatling.http.Predef._
import scala.concurrent.duration._

class DownstreamSimulation extends Simulation {

  val httpConf = http.baseURL("http://localhost:9000")

  val scn = scenario("DownstreamSimulation")
    .exec(http("OnlyRequest").get("/"))
//    .pause(1 second)

  setUp(
    scn.inject(rampUsers(100) over (10 seconds))
  ).assertions(
    // no FiniteDuration!
    global.responseTime.min.greaterThan(2000),
    global.responseTime.percentile2.lessThan(2020),
    global.responseTime.max.lessThan(2050)
  ).protocols(httpConf)
}
