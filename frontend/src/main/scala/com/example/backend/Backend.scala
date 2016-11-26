package com.example.backend

import akka.actor.{Actor, ActorLogging, Props}

object Backend {
  def props = Props(new Backend)
}

class Backend extends Actor with ActorLogging {

  import com.example.backend.BackendApi._

  override def receive = {
    case Greet(name: String) => {
      log.info("Greeting {}", name)
      sender() ! Greeting(s"Hello, $name")
    }
  }

}
