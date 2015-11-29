package actors

import akka.actor.{Actor, Props}
import akka.contrib.pattern.DistributedPubSubExtension
import akka.contrib.pattern.DistributedPubSubMediator.Publish

import scala.collection.mutable

object WordStoreActor {

  case object RequestUpdate
  case class SendUpdate(words: Seq[String])
  case class WordUpdate(words: Seq[(String, Int)])

  def props() = Props(new WordStoreActor())
}

class WordStoreActor() extends Actor {

  import WordStoreActor._

  private val mediator = DistributedPubSubExtension(context.system).mediator

  private val words = mutable.Map[String, Int](
      "Scala" -> 1,
      "Scala.JS" -> 1,
      "Play" -> 1,
      "Akka" -> 1,
      "SBT" -> 1
    ).withDefaultValue(0)

  def receive = {
    case RequestUpdate => {
      sender() ! WordUpdate(normalisedWords)
    }
    case SendUpdate(newWords) => {
      newWords foreach { w =>
        words(w) += 1
      }
      mediator ! Publish("word-updates", WordUpdate(normalisedWords))
    }
  }

  private def normalisedWords = {
    val maxValue = words.values.max
    words.toSeq.map { case (word: String, count: Int) => (word, count*100/maxValue) }
  }
}
