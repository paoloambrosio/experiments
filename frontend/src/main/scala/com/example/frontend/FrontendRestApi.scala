package com.example.frontend

import akka.http.scaladsl.server.Directives._
import com.example.backend.BackendApi
import de.heikoseeberger.akkahttpjackson.JacksonSupport

trait FrontendRestApi extends JacksonSupport with BackendApi {

  def routes = (path("greet" / Segment) & get) { name =>
    complete(greet(name))
  }
}
