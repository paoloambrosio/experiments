package com.example

import akka.actor.ActorSystem
import akka.event.LoggingAdapter
import com.typesafe.config.ConfigFactory

trait ExampleApp extends ClusterSeedNodesConfig {

  val actorSystemName = "example"
  val config = ConfigFactory.load()
  implicit val system = ActorSystem(actorSystemName, configWithSeedNodes)
  val log: LoggingAdapter = system.log
}
