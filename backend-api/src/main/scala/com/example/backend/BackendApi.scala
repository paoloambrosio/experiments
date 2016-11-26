package com.example.backend

import akka.actor.ActorRef
import akka.pattern.ask
import akka.util.Timeout

import scala.concurrent.Future

trait BackendApi {

  import BackendApi._

  def backend: ActorRef
  implicit def backendTimeout: Timeout

  def greet(name: String): Future[Greeting] = backend.ask(Greet(name)).mapTo[Greeting]
}

object BackendApi {

  case class Greet(name: String)
  case class Greeting(message: String)
}
