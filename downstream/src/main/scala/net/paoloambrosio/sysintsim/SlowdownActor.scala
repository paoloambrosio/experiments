package net.paoloambrosio.sysintsim

import akka.actor.{Actor, Props}
import akka.event.Logging

import scala.collection.mutable.{Seq => MutableSeq}
import scala.compat.Platform
import scala.concurrent.duration.FiniteDuration

object SlowdownActor {

  type Distribution = Int => FiniteDuration

  def props(windowSizeMs: Int, maxRequests: Int, distribution: Distribution) = Props(new SlowdownActor(windowSizeMs, maxRequests, distribution))

}

/**
 *
 * @param windowSizeMs max size of window to keep
 * @param maxRequests max requests to keep
 * @param distribution returns the time a request should take based on the
 *                 current load
 */
class SlowdownActor(windowSizeMs: Int, maxRequests: Int, distribution: SlowdownActor.Distribution) extends Actor {

  val counter = MutableSeq[Long]()

  override def receive = {
    case _ => {
      updateCounter
      sender ! distribution(counter.length)
    }
  }
  
  def updateCounter: Unit = {
    val epochMs = Platform.currentTime
    (epochMs +: (counter.filter(_ >= epochMs - windowSizeMs))).take(maxRequests)
  }
}