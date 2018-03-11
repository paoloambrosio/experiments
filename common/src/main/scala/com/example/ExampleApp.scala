package com.example

import akka.actor.ActorSystem
import akka.event.LoggingAdapter
import akka.management.AkkaManagement
import akka.management.cluster.bootstrap.ClusterBootstrap
import com.typesafe.config.ConfigFactory

trait ExampleApp {

  val actorSystemName = "example"
  val config = ConfigFactory.load()
  implicit val system = ActorSystem(actorSystemName, config)
  val log: LoggingAdapter = system.log

  AkkaManagement(system).start()
  ClusterBootstrap(system).start()
}
