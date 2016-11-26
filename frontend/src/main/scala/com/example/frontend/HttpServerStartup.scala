package com.example.frontend

import akka.actor.ActorSystem
import akka.event.LoggingAdapter
import akka.http.scaladsl.Http
import akka.http.scaladsl.server.Route
import akka.stream.ActorMaterializer

import scala.concurrent.ExecutionContext

trait HttpServerStartup {

  implicit def system: ActorSystem
  implicit def executionContext: ExecutionContext
  implicit def log: LoggingAdapter

  def routes: Route

  def startHttpServer(interface: String, port: Int)= {
    implicit val materializer = ActorMaterializer()
    Http().bindAndHandle(routes, interface, port).map { _ =>
      log.info("HTTP server started on {}:{}", interface, port)
    }.onFailure {
      case ex: Exception =>
        log.error(ex, "Failed to bind to {}:{}!", interface, port)
        system.terminate()
    }
  }

}
