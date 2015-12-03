package controllers

import actors.WordStoreActor.{SendUpdate, WordUpdate}
import actors.{WordStoreActor, WordWebSocketActor}
import play.api.data.Form
import play.api.libs.json._
import play.api.mvc.WebSocket.FrameFormatter
import play.api.mvc._
import play.api.Play.current
import play.api.data._
import play.api.data.Forms._
import play.api.Play.current
import play.api.i18n.Messages.Implicits._

import akka.actor._
import javax.inject._


@Singleton
class Application @Inject() (system: ActorSystem) extends Controller {

  lazy val wordStore = system.actorOf(WordStoreActor.props, "word-store-actor")

  val wordForm = Form(
    single(
      "word" -> nonEmptyText
    )
  )

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
    wordStore ! SendUpdate(Seq(word))
    NoContent
  }

  def vote = Action { implicit request =>
    Ok(views.html.vote(wordForm))
  }

  def makeVote = Action { implicit request =>
    wordForm.bindFromRequest.fold(
      formWithErrors => {
        BadRequest(views.html.vote(formWithErrors))
      },
      word => {
        wordStore ! SendUpdate(Seq(word))
        Redirect("/vote").flashing(
          "success" -> s"You voted for $word!"
        )
      }
    )
  }
}
