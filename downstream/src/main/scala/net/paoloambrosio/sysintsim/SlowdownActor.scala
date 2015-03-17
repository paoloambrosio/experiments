package net.paoloambrosio.sysintsim

import akka.actor.{Actor, Props}
import akka.event.Logging

import scala.concurrent.duration.FiniteDuration

object SlowdownActor {

  type Strategy = Int => FiniteDuration

  def props(strategy: Strategy) = Props(new SlowdownActor(strategy))

}

class SlowdownActor(strategy: SlowdownActor.Strategy) extends Actor {

  val log = Logging(context.system, this)
  var count = 0

  override def receive = {
    case _ => {
      count += 1
      sender ! strategy(count)
    }
  }
}