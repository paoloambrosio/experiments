package actors

import akka.actor.{Actor, ActorRef, Props}
import akka.contrib.pattern.DistributedPubSubExtension
import akka.contrib.pattern.DistributedPubSubMediator.{Subscribe, SubscribeAck}

object WordWebSocketActor {

  def props(out: ActorRef) = Props(new WordWebSocketActor(out))
}

class WordWebSocketActor(out: ActorRef) extends Actor {

  import actors.WordStoreActor.WordUpdate

  val mediator = DistributedPubSubExtension(context.system).mediator

  mediator ! Subscribe("word-updates", self)

  def receive = {
    case SubscribeAck(Subscribe("word-updates", None, `self`)) =>
      context become ready
  }

  def ready: Actor.Receive = {
    case wu: WordUpdate => out ! wu.testMessage
  }
}