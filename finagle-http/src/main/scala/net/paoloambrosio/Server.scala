import com.twitter.finagle.{Http, Service, Thrift, http}
import com.twitter.io.Buf
import com.twitter.util.{Await, Future}
import net.paoloambrosio.greeting.GreetingService

object Server extends App {

  val client = Thrift.client.withLabel("greeting-service").build[GreetingService[Future]]("finagle-thrift:8080")

  val service = new Service[http.Request, http.Response] {
    def apply(req: http.Request): Future[http.Response] =
      client.greet("World").map(greeting =>
        http.Response(req.version, http.Status.Ok).content(Buf.Utf8(greeting))
      )
  }

  val server = Http.server.withLabel("finagle-http").serve(":8080", service)
  Await.ready(server)
}
