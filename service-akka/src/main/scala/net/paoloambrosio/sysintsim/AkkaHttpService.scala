package net.paoloambrosio.sysintsim

import akka.actor.ActorSystem
import akka.event.{Logging, LoggingAdapter}
import akka.http.scaladsl.Http
import akka.http.scaladsl.model.{HttpRequest, Uri, HttpResponse}
import akka.http.scaladsl.model.StatusCodes._
import akka.http.scaladsl.Http.OutgoingConnection
import akka.http.scaladsl.server.Directives._
import akka.http.scaladsl.server.ExceptionHandler
import akka.http.scaladsl.unmarshalling.Unmarshal
import akka.pattern.after
import akka.pattern.ask
import akka.stream.{ActorFlowMaterializer, FlowMaterializer}
import akka.stream.scaladsl._
import akka.util.Timeout
import com.typesafe.config.{Config, ConfigFactory}
import net.paoloambrosio.sysintsim.SlowdownActor.ComputeSlowdown
import net.paoloambrosio.sysintsim.slowdown.SlowdownProviderFactory
import org.jmxtrans.embedded.config.ConfigurationParser

import scala.concurrent.duration._
import scala.concurrent.{ExecutionContextExecutor, Future}

trait Service {
  implicit val system: ActorSystem
  implicit def executor: ExecutionContextExecutor
  implicit val materializer: FlowMaterializer

  val config: Config
  val logger: LoggingAdapter
  val downstreamConnectionFlow: Option[Flow[HttpRequest, HttpResponse, Any]]

  lazy val slowdownActor = {
    val slowdownStrategy = config.getString("config.slowdown-strategy")
    val slowdownProvider = SlowdownProviderFactory.notThreadSafe(slowdownStrategy)
    system.actorOf(SlowdownActor.props(slowdownProvider))
  }

  implicit val timeout = Timeout(10 seconds)

  val routes = {
    get {
      complete {
        for {
          d <- (slowdownActor ? ComputeSlowdown).mapTo[FiniteDuration]
          response <- callDownstream
        } yield after(d, using = system.scheduler)(Future.successful(response))
      }
    }
  }

  def callDownstream(): Future[String] = {
    downstreamConnectionFlow match {
      case Some(dcf) => Source.single(HttpRequest(uri = Uri("/"))).via(dcf).runWith(Sink.head)
        .flatMap(Unmarshal(_).to[String])
      case None => Future.successful("success")
    }
  }

  implicit def exceptionHandler: ExceptionHandler = ExceptionHandler {
    case t: Throwable => {
      logger.error(t, "Exception caught")
      complete(HttpResponse(InternalServerError, entity = "failure"))
    }
  }
}

object AkkaHttpService extends App with Service {
  override implicit val system = ActorSystem()
  override implicit val executor = system.dispatcher
  override implicit val materializer = ActorFlowMaterializer()

  override val config = ConfigFactory.load()
  override val logger = Logging(system, getClass)

  new ConfigurationParser().newEmbeddedJmxTrans("classpath:jmxtrans.json").start()

  Http().bindAndHandle(handler=routes, interface=config.getString("server.address"), port=config.getInt("server.port"))

  override val downstreamConnectionFlow = {
    // FIXME cannot believe the config library is this bad
    val host = config.getString("service.downstream.host")
    if (host.isEmpty) {
      None
    }else {
      val port = config.getInt("service.downstream.port")
      Some(Http(system).outgoingConnection(host, port))
    }
  }
}
