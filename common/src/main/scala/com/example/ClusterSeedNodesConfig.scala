package com.example

import com.typesafe.config.{Config, ConfigValueFactory}

import scala.collection.JavaConversions._

trait ClusterSeedNodesConfig {

  def actorSystemName: String
  def config: Config

  def configWithSeedNodes: Config = {
    config.withValue("akka.cluster.seed-nodes", ConfigValueFactory.fromIterable(seedNodes))
  }

  private def seedNodes: List[String] = {
    val remoteInterface = config.getString("app.remote.interface")
    val remotePort = config.getInt("app.remote.port")
    if (config.hasPath("app.cluster.seeds")) {
      config.getString("app.cluster.seeds")
        .split(',').toList
        .map(toAkkaNodeRef(remotePort))
        .flatten
    } else if (remotePort > 0) {
      List(toAkkaNodeRef(remoteInterface, remotePort))
    } else {
      List.empty
    }
  }

  private def toAkkaNodeRef(defaultPort: Int)(peer: String): Option[String] = {
    peer.trim.split(':') match {
      case Array(host, port) =>
        Some(toAkkaNodeRef(host, port.toInt))
      case Array(host) =>
        if (defaultPort == 0) None
        else Some(toAkkaNodeRef(host, defaultPort))
    }
  }

  private def toAkkaNodeRef(host: String, port: Int) = {
    s"akka.tcp://$actorSystemName@$host:$port"
  }

}
