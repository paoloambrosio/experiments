package net.paoloambrosio.d3js

import org.singlespaced.d3js.LayoutObject

import scala.language.implicitConversions
import scala.scalajs.js
import scala.scalajs.js.annotation.ScalaJSDefined

package object cloud {

  @ScalaJSDefined
  class Word(val text: String, val size: Int) extends js.Object

  @js.native
  trait WordTag extends js.Object {
    val text: String = js.native
    val size: Int = js.native
    val x: Int = js.native
    val y: Int = js.native
    val rotate: Int = js.native
  }

  implicit def layoutObjectToCloud(lo: LayoutObject): LayoutObjectCloudExtension =
    lo.asInstanceOf[LayoutObjectCloudExtension]
}
