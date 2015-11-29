package actors

import akka.actor.{Actor, Props}
import akka.contrib.pattern.DistributedPubSubExtension
import akka.contrib.pattern.DistributedPubSubMediator.Publish

object WordStoreActor {

  case class WordUpdate(testMessage: String)

  def props() = Props(new WordStoreActor())
}

class WordStoreActor() extends Actor {

  import WordStoreActor._

  val mediator = DistributedPubSubExtension(context.system).mediator

  def receive = {
    case msg: String =>
      val wu = WordUpdate(s"Update: $msg")
      mediator ! Publish("word-updates", wu)
  }
}
