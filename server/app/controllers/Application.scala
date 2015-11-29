package controllers

import actors.WordStoreActor.WordUpdate
import actors.{WordStoreActor, WordWebSocketActor}
import play.api.libs.json._
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

  implicit val wordUpdateFormat = new Format[WordUpdate] {
    override def writes(wu: WordUpdate): JsValue = Json.toJson(wu.words.map(w => Json.obj(
      "text" -> w._1,
      "size" -> w._2
    )))
    override def reads(json: JsValue): JsResult[WordUpdate] = ???
  }
  implicit val wordUpdateFrameFormatter = FrameFormatter.jsonFrame[WordUpdate]

  def words = WebSocket.acceptWithActor[String, WordUpdate] { request => out =>
    WordWebSocketActor.props(out, wordStore)
  }

  def test(word: String) = Action {
    wordStore ! word
    NoContent
  }
}
