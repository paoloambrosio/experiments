package com.example.frontend

import akka.actor.{ActorRef, ActorSystem}
import akka.event.LoggingAdapter
import akka.util.Timeout
import com.example.backend.Backend
import com.typesafe.config.ConfigFactory

import scala.concurrent.ExecutionContext
import scala.concurrent.duration._

object FrontendApp extends App
  with HttpServerStartup with FrontendRestApi {

  val actorSystemName = "example"
  val config = ConfigFactory.load()

  implicit val system = ActorSystem(actorSystemName, config)
  override implicit val executionContext: ExecutionContext = system.dispatcher
  val log: LoggingAdapter = system.log

  override val backend: ActorRef = system.actorOf(Backend.props, "backend")
  override implicit val backendTimeout: Timeout = 5 seconds

  val interface = config.getString("app.http.interface")
  val port = config.getInt("app.http.port")
  startHttpServer(interface, port)
}
