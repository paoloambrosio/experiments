package net.paoloambrosio

import com.twitter.finagle.Thrift
import com.twitter.util.{Await, Future}
import net.paoloambrosio.greeting.GreetingService

object Server extends App {

//  val openTracing = {
//    val tracer = Configuration.fromEnv.getTracer
//    new OpenTracingHttpFilter(tracer, true)
//  }

  val service = new GreetingService[Future] {
    override def greet(name: String): Future[String] = Future(s"Hello, $name!")
  }

//  val server = Http.serve(":8080", openTracing andThen service)
  val server = Thrift.server.serveIface(":8080", service)
  Await.ready(server)
}
