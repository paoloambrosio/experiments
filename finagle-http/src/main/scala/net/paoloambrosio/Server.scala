import com.twitter.finagle.{Http, Service}
import com.twitter.finagle.http
import com.twitter.io.Buf
import com.twitter.util.{Await, Future}
import com.uber.jaeger.Configuration
import io.opentracing.contrib.finagle.OpenTracingHttpFilter

object Server extends App {

  val openTracing = {
    val tracer = Configuration.fromEnv.getTracer
    new OpenTracingHttpFilter(tracer, isServerFilter = true)
  }

  val service = new Service[http.Request, http.Response] {
    def apply(req: http.Request): Future[http.Response] =
      Future.value(
        http.Response(req.version, http.Status.Ok).content(Buf.Utf8("Hello, World!"))
      )
  }

  val server = Http.serve(":8080", openTracing andThen service)
  Await.ready(server)
}
