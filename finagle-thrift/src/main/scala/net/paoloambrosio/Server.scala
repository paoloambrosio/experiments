package net.paoloambrosio

import com.twitter.finagle.Thrift
import com.twitter.util.{Await, Future}
import net.paoloambrosio.greeting.GreetingService

object Server extends App {

  val service = new GreetingService[Future] {
    override def greet(name: String): Future[String] = Future(s"Hello, $name!")
  }

  val server = Thrift.server.withLabel("finagle-thrift").serveIface(":8080", service)
  Await.ready(server)
}
