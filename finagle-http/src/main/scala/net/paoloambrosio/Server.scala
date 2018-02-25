import com.twitter.finagle.{Http, Service, Thrift, http}
import com.twitter.io.Buf
import com.twitter.util.{Await, Future}
import com.uber.jaeger.Configuration
import io.opentracing.contrib.finagle.OpenTracingHttpFilter
import net.paoloambrosio.greeting.GreetingService

object Server extends App {

  val openTracing = {
    val tracer = Configuration.fromEnv.getTracer
    new OpenTracingHttpFilter(tracer, isServerFilter = true)
  }

  val client = Thrift.client.build[GreetingService[Future]]("finagle-thrift:8080")

  val service = new Service[http.Request, http.Response] {
    def apply(req: http.Request): Future[http.Response] =
      client.greet("World").map(greeting =>
        http.Response(req.version, http.Status.Ok).content(Buf.Utf8(greeting))
      )
  }

  val server = Http.serve(":8080", openTracing andThen service)
  Await.ready(server)
}
