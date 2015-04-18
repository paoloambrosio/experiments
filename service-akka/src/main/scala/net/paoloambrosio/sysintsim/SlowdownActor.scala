package net.paoloambrosio.sysintsim

import akka.actor.{Actor, Props}
import net.paoloambrosio.sysintsim.slowdown.SlowdownProvider
import scala.concurrent.duration._

object SlowdownActor {

  object ComputeSlowdown

  def props(slowdownProvider: SlowdownProvider) = Props(new SlowdownActor(slowdownProvider))

}

class SlowdownActor(slowdownProvider: SlowdownProvider) extends Actor {

  import SlowdownActor._

  override def receive = {
    case ComputeSlowdown => {
      sender ! (slowdownProvider.computeSlowdown() millis)
    }
  }

}