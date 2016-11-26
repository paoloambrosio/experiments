package com.example.backend

import com.example.ExampleApp

object BackendApp extends App with ExampleApp {

  system.actorOf(Backend.props, "backend")

  log.info("Backend Service started")
}
