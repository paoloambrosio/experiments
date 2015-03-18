package net.paoloambrosio.sysintsim

import akka.actor.{Actor, Props}
import akka.event.Logging

import scala.collection.Seq
import scala.compat.Platform
import scala.concurrent.duration.FiniteDuration

object SlowdownActor {

  type Distribution = Int => FiniteDuration

  def props(maxRequests: Int, distribution: Distribution) = Props(new SlowdownActor(maxRequests, distribution))

}

/**
 *
 * @param maxRequests max requests to keep
 * @param distribution returns the time a request should take based on the
 *                 current load
 */
class SlowdownActor(maxRequests: Int, distribution: SlowdownActor.Distribution) extends Actor {

  var counter = Seq[Long]()

  override def receive = {
    case _ => {
      val rps = updateCounter
      sender ! distribution(rps)
    }
  }
  
  def updateCounter: Int = {
    val epochMs = Platform.currentTime
    counter = (epochMs +: (counter.filter(_ > epochMs - 1000))).take(maxRequests)
    counter.length
  }
}