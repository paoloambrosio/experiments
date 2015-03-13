package net.paoloambrosio.sysintsim

import akka.actor.ActorSystem
import akka.event.{Logging, LoggingAdapter}
import akka.http.Http
import akka.http.marshalling.ToResponseMarshallable
import akka.http.model.HttpResponse
import akka.http.model.StatusCodes._
import akka.http.server.Directives._
import akka.http.server.ExceptionHandler
import akka.stream.FlowMaterializer
import com.typesafe.config.{Config, ConfigFactory}

import scala.concurrent.ExecutionContextExecutor

trait Service {
  implicit val system: ActorSystem
  implicit def executor: ExecutionContextExecutor
  implicit val materializer: FlowMaterializer

  def config: Config
  val logger: LoggingAdapter

  val routes = {
    get {
      complete {
        ToResponseMarshallable("not_implemented")
      }
    }
  }

  implicit def exceptionHandler = ExceptionHandler {
    case t: Throwable => complete(HttpResponse(InternalServerError, entity = "exception"))
  }
}

object AkkaHttpDownstream extends App with Service {
  override implicit val system = ActorSystem()
  override implicit val executor = system.dispatcher
  override implicit val materializer = FlowMaterializer()

  override val config = ConfigFactory.load()
  override val logger = Logging(system, getClass)

  Http().bind(interface = config.getString("http.interface"), port = config.getInt("http.port")).startHandlingWith(routes)
}
