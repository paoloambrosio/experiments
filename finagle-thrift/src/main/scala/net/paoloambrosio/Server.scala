package net.paoloambrosio

import com.twitter.finagle.Thrift
import com.twitter.util.Await
import net.paoloambrosio.greeting.GreetingService

import scala.concurrent.Future

object Server extends App {

//  val openTracing = {
//    val tracer = Configuration.fromEnv.getTracer
//    new OpenTracingHttpFilter(tracer, true)
//  }

  val service = new GreetingService[Future] {
    override def greet(name: String): Future[String] = Future.successful(s"Hello, $name!")
  }

//  val server = Http.serve(":8080", openTracing andThen service)
  val server = Thrift.server.serveIface(":8080", service)
  Await.ready(server)
}
