import akka.actor.{Props, ActorSystem}
import com.typesafe.config.{ConfigValueFactory, ConfigFactory}

import scala.collection.JavaConversions._

object ClusterApp extends App {

  val (seedHostPort, ports) = args.partition(_.contains(":"))
  val localClusterPort = ports.headOption.map(_.toInt).getOrElse(0)
  val seedHostPortOrSelf = if (seedHostPort.isEmpty) Array(s"127.0.0.1:${localClusterPort}") else seedHostPort
  val seedNodes = seedHostPortOrSelf.map(hostPort => s"akka.tcp://ClusterSystem@${hostPort}").toIterable

  val config = ConfigFactory.load()
    .withValue("akka.cluster.seed-nodes", ConfigValueFactory.fromIterable(asJavaIterable(seedNodes)))
    .withValue("akka.remote.netty.tcp.port", ConfigValueFactory.fromAnyRef(localClusterPort))

  Console.out.println(Console.CYAN_B + Console.BLACK + s"=== Starting on port ${localClusterPort} ===" + Console.RESET)

  val system = ActorSystem("ClusterSystem", config)
  system.actorOf(Props[ClusterListenerActor], name = "clusterListener")
}
