package net.paoloambrosio.d3js.cloud

import scala.scalajs.js
import org.singlespaced.d3js.LayoutObject

/*
 * JS monkey patching handled with pimp-my-library scala pattern
 * http://www.scala-js.org/doc/interoperability/facade-types.html
 */
@js.native
trait LayoutObjectCloudExtension extends LayoutObject {

 def cloud(): CloudLayout = js.native
}


