package net.paoloambrosio.sysintsim

import akka.actor.ActorSystem
import akka.event.{Logging, LoggingAdapter}
import akka.http.Http
import akka.http.model.HttpResponse
import akka.http.model.StatusCodes._
import akka.http.server.Directives._
import akka.http.server.ExceptionHandler
import akka.pattern.after
import akka.stream.FlowMaterializer
import com.typesafe.config.{Config, ConfigFactory}
import org.jmxtrans.embedded.config.ConfigurationParser

import scala.concurrent.duration._
import scala.concurrent.{ExecutionContextExecutor, Future}

trait Service {
  implicit val system: ActorSystem
  implicit def executor: ExecutionContextExecutor
  implicit val materializer: FlowMaterializer

  def config: Config
  val logger: LoggingAdapter

  val routes = {
    get {
      complete {
        after(2 second, using = system.scheduler)(Future("success"))
      }
    }
  }

  implicit def exceptionHandler = ExceptionHandler {
    case t: Throwable => complete(HttpResponse(InternalServerError, entity = "failure"))
  }
}

object AkkaHttpDownstream extends App with Service {
  override implicit val system = ActorSystem()
  override implicit val executor = system.dispatcher
  override implicit val materializer = FlowMaterializer()

  override val config = ConfigFactory.load()
  override val logger = Logging(system, getClass)

  private val jmxTrans = {
    val jt = new ConfigurationParser().newEmbeddedJmxTrans("classpath:jmxtrans.json")
    jt.start()
    //sys.addShutdownHook { jt.stop() }
    jt
  }

  Http().bind(interface = config.getString("http.interface"), port = config.getInt("http.port")).startHandlingWith(routes)

}
