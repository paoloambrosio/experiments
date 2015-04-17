package net.paoloambrosio.sysintsim

import akka.actor.{Actor, Props}
import net.paoloambrosio.sysintsim.slowdown.SlowdownProvider
import scala.concurrent.duration._

object SlowdownActor {

  def props(slowdownProvider: SlowdownProvider) = Props(new SlowdownActor(slowdownProvider))

}

class SlowdownActor(slowdownProvider: SlowdownProvider) extends Actor {

  override def receive = {
    case _ => {
      val slowdown = slowdownProvider.computeSlowdown() millis
      sender ! slowdown
    }
  }

}