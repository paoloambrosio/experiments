import akka.actor.ActorSystem
import akka.event.{Logging, LoggingAdapter}
import akka.http.scaladsl.Http
import akka.http.scaladsl.model.headers.{HttpCredentials, BasicHttpCredentials}
import akka.http.scaladsl.server.Directives._
import akka.http.scaladsl.server.directives.{AuthenticationDirective, AuthenticationResult}
import akka.stream.{ActorMaterializer, Materializer}
import com.typesafe.config.{Config, ConfigFactory}

import scala.concurrent.{ExecutionContextExecutor, Future}


trait Service {
  implicit val system: ActorSystem
  implicit def executor: ExecutionContextExecutor
  implicit val materializer: Materializer

  def config: Config
  val logger: LoggingAdapter

  /*
   * correct   -> 200 OK
   * incorrect -> 401 Unauthorized
   * failed    -> 500 Internal Server Error
   */
  val routes = {
    logRequestResult("akka-http") {
      authenticated(myCheckCredentials) { username =>
        get {
          complete(s"Resource content for '$username'")
        }
      }
    }
  }

  /**
   * Returns failed future is call failed. A successful future would contain if the credentials were correct or not.
   */
  type UserAuthenticator[T] = (String, String) => Future[Option[T]]

  def authenticated[T](f: UserAuthenticator[T]): AuthenticationDirective[T] = {
    val failedCredentials = AuthenticationResult.failWithChallenge(challengeFor("myrealm"))

    def authenticator: Option[HttpCredentials] ⇒ Future[AuthenticationResult[T]] = {
      case Some(BasicHttpCredentials(username, password)) => f(username, password) map {
        case Some(u) => AuthenticationResult.success(u)
        case None => failedCredentials
      }
      case _ => Future.successful(failedCredentials)
    }

    authenticateOrRejectWithChallenge(authenticator)
  }

  def myCheckCredentials: UserAuthenticator[String] = (username, password) => password match {
    case "correct" => Future.successful(Some(username))
    case "failed" => Future.failed(new Throwable("Something went wrong"))
    case _ => Future.successful(None)
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
