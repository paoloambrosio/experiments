package com.example.frontend

import akka.actor.ActorRef
import akka.routing.RoundRobinGroup
import akka.util.Timeout
import com.example.{BackendPathsConfig, ExampleApp}

import scala.concurrent.ExecutionContext
import scala.concurrent.duration._

object FrontendApp extends App with ExampleApp with BackendPathsConfig
  with HttpServerStartup with FrontendRestApi {

  override implicit val executionContext: ExecutionContext = system.dispatcher

  override val backend: ActorRef = system.actorOf(RoundRobinGroup(backendPaths()).props(), "backend")
  override implicit val backendTimeout: Timeout = 5 seconds

  val interface = config.getString("app.http.interface")
  val port = config.getInt("app.http.port")
  startHttpServer(interface, port)
}
