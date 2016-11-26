package com.example

import com.typesafe.config.Config

trait BackendPathsConfig {

  def actorSystemName: String
  def config: Config

  def backendPaths() = remoteNodesAddress.map(node => s"$node/user/backend")

  private def remoteNodesAddress: List[String] = {
    val remoteInterface = config.getString("app.remote.interface")
    val remotePort = config.getInt("app.remote.port")
    if (config.hasPath("app.backend.nodes")) {
      config.getString("app.backend.nodes")
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
