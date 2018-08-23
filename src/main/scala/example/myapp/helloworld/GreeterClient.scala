package example.myapp.helloworld

import akka.{Done, NotUsed}
import akka.actor.ActorSystem
import akka.grpc.GrpcClientSettings
import akka.stream.ActorMaterializer
import akka.stream.scaladsl.Source
import example.myapp.helloworld.grpc._

import scala.concurrent.{Await, Future}
import scala.concurrent.duration._
import scala.util.Try

object GreeterClient {

  def main(args: Array[String]): Unit = {

    implicit val sys = ActorSystem()
    implicit val mat = ActorMaterializer()
    implicit val ec = sys.dispatcher

    val client = new GreeterServiceClient(GrpcClientSettings.connectToServiceAt("localhost", 8080))

    def singleRequestReply(): Unit = {
      sys.log.info("Performing request")
      val reply = client.sayHello(HelloRequest("Alice"))
      println(s"got single reply: ${Await.result(reply, 5.seconds).message}")
    }

    def streamingRequest(): Unit = {
      val requests = List("Alice", "Bob", "Peter").map(HelloRequest.apply)
      val reply = client.itKeepsTalking(Source(requests))
      println(s"got single reply for streaming requests: ${Await.result(reply, 5.seconds).message}")
    }

    def streamingReply(): Unit = {
      val responseStream = client.itKeepsReplying(HelloRequest("Alice"))
      val done: Future[Done] =
        responseStream.runForeach(reply => println(s"got streaming reply: ${reply.message}"))
      Await.ready(done, 1.minute)
    }

    def streamingRequestReply(): Unit = {
      val requestStream: Source[HelloRequest, NotUsed] =
        Source
          .tick(100.millis, 1.second, "tick")
          .zipWithIndex
          .map { case (_, i) => i }
          .map(i => HelloRequest(s"Alice-$i"))
          .take(10)
          .mapMaterializedValue(_ => NotUsed)

      val responseStream: Source[HelloReply, NotUsed] = client.streamHellos(requestStream)
      val done: Future[Done] =
        responseStream.runForeach(reply => println(s"got streaming reply: ${reply.message}"))
      Await.ready(done, 1.minute)
    }

    singleRequestReply()
    streamingRequest()
    streamingReply()
    streamingRequestReply()

    sys.scheduler.schedule(1.second, 1.second) {
      Try(singleRequestReply())
    }
  }

}