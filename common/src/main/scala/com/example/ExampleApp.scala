package com.example

import akka.actor.ActorSystem
import akka.event.LoggingAdapter
import com.typesafe.config.ConfigFactory

trait ExampleApp {

  val actorSystemName = "example"
  val config = ConfigFactory.load()
  implicit val system = ActorSystem(actorSystemName, config)
  val log: LoggingAdapter = system.log
}
