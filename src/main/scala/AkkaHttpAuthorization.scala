import akka.actor.ActorSystem
import akka.event.{Logging, LoggingAdapter}
import akka.http.scaladsl.Http
import akka.http.scaladsl.server.Directives._
import akka.http.scaladsl.server.directives.UserCredentials
import akka.stream.{ActorMaterializer, Materializer}
import com.typesafe.config.{Config, ConfigFactory}

import scala.concurrent.ExecutionContextExecutor


trait Service {
  implicit val system: ActorSystem
  implicit def executor: ExecutionContextExecutor
  implicit val materializer: Materializer

  def config: Config
  val logger: LoggingAdapter

  val routes = {
    logRequestResult("akka-http") {
      authenticated { username =>
        get {
          complete(s"Resource content for '$username'")
        }
      }
    }
  }

  def authenticated = authenticateBasic("myrealm", passwordMatchCorrect)

  def passwordMatchCorrect: Authenticator[String] = {
    case uc: UserCredentials.Provided => if (uc.verifySecret("correct")) Some(uc.username) else None
    case UserCredentials.Missing => None
  }
}

object AkkaHttpAuthorization extends App with Service {
  override implicit val system = ActorSystem()
  override implicit val executor = system.dispatcher
  override implicit val materializer = ActorMaterializer()

  override val config = ConfigFactory.load()
  override val logger = Logging(system, getClass)

  Http().bindAndHandle(routes, config.getString("http.interface"), config.getInt("http.port"))
}
