package controllers

import actors.WordStoreActor.WordUpdate
import actors.{WordStoreActor, WordWebSocketActor}
import play.api.mvc.WebSocket.FrameFormatter
import play.api.mvc._
import play.api.Play.current

import akka.actor._
import javax.inject._


@Singleton
class Application @Inject() (system: ActorSystem) extends Controller {

  lazy val wordStore = system.actorOf(WordStoreActor.props, "word-store-actor")

  def index = Action {
    Ok(views.html.index())
  }

  def words = WebSocket.acceptWithActor[String, String] { request => out =>
    WordWebSocketActor.props(out)
  }

  def test(word: String) = Action {
    wordStore ! word
    NoContent
  }
}
