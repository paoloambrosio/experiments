package example.myapp.helloworld

import java.io.{ByteArrayOutputStream, FileInputStream, InputStream}
import java.nio.file.Files
import java.nio.file.Paths
import java.security.KeyFactory
import java.security.KeyStore
import java.security.SecureRandom
import java.security.cert.CertificateFactory
import java.security.spec.PKCS8EncodedKeySpec
import java.util.Base64

import javax.net.ssl.KeyManagerFactory
import javax.net.ssl.SSLContext

import scala.concurrent.ExecutionContext
import scala.concurrent.Future
import akka.actor.ActorSystem
import akka.http.scaladsl.Http
import akka.http.scaladsl.HttpsConnectionContext
import akka.http.scaladsl.model.StatusCodes
import akka.http.scaladsl.model.HttpRequest
import akka.http.scaladsl.model.HttpResponse
import akka.stream.ActorMaterializer
import akka.stream.Materializer
import com.typesafe.config.ConfigFactory

import example.myapp.helloworld.grpc._

object GreeterServer {

  def main(args: Array[String]): Unit = {
    // important to enable HTTP/2 in ActorSystem's config
    val conf = ConfigFactory.parseString("akka.http.server.preview.enable-http2 = on")
      .withFallback(ConfigFactory.defaultApplication())
    implicit val sys: ActorSystem = ActorSystem("HelloWorld", conf)
    implicit val mat: Materializer = ActorMaterializer()
    implicit val ec: ExecutionContext = sys.dispatcher

    val service: HttpRequest => Future[HttpResponse] = GreeterServiceHandler(new GreeterServiceImpl(mat))
      .orElse { case _  => Future.successful(HttpResponse(StatusCodes.NotFound)) }

    Http().bindAndHandleAsync(
      service,
      interface = "127.0.0.1",
      port = 8080,
      connectionContext = serverHttpContext()
    )
      .foreach { binding =>
        println(s"gRPC server bound to: ${binding.localAddress}")
      }
  }

  private def serverHttpContext(): HttpsConnectionContext = {
    val keyEncoded = read(GreeterServer.getClass.getResourceAsStream("/certs/key.pem"))
      .replace("-----BEGIN PRIVATE KEY-----\n", "")
      .replace("-----END PRIVATE KEY-----\n", "")
      .replace("\n", "")

    val decodedKey = Base64.getDecoder.decode(keyEncoded)

    val spec = new PKCS8EncodedKeySpec(decodedKey)

    val kf = KeyFactory.getInstance("RSA")
    val privateKey = kf.generatePrivate(spec)

    val fact = CertificateFactory.getInstance("X.509")
    //    val is = new FileInputStream(TestUtils.loadCert("server1.pem"))
    val cer = fact.generateCertificate(GreeterServer.getClass.getResourceAsStream("/certs/certificate.pem"))

    val ks = KeyStore.getInstance("PKCS12")
    ks.load(null)
    ks.setKeyEntry("private", privateKey, Array.empty, Array(cer))

    val keyManagerFactory = KeyManagerFactory.getInstance("SunX509")
    keyManagerFactory.init(ks, null)

    val context = SSLContext.getInstance("TLS")
    context.init(keyManagerFactory.getKeyManagers, null, new SecureRandom)

    new HttpsConnectionContext(context)
  }

  private def read(in: InputStream): String = {
    val bytes: Array[Byte] = {
      val baos = new ByteArrayOutputStream(math.max(64, in.available()))
      val buffer = Array.ofDim[Byte](32 * 1024)

      var bytesRead = in.read(buffer)
      while (bytesRead >= 0) {
        baos.write(buffer, 0, bytesRead)
        bytesRead = in.read(buffer)
      }
      baos.toByteArray
    }
    new String(bytes, "UTF-8")
  }

}
