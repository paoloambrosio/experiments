package net.paoloambrosio.d3js.cloud

import scala.scalajs.js

/*
 * Welcome to rubbish coding practices of calling getter and setter in the same way.
 */
@js.native
trait CloudLayout extends js.Object {

  def size(array: js.Array[Int]): this.type = js.native

  def words(list: js.Array[Word]): this.type = js.native
  def words(): js.Array[Word] = js.native

  def padding(value: Int): this.type = js.native

  def font(value: String): this.type = js.native

  def fontSize(f: js.Function1[Word, Int]): this.type = js.native

  def rotate(f: js.Function1[Word, Int]): this.type = js.native

  def on(event: String, handler: js.Function): this.type = js.native

  def start(): this.type = js.native

  def stop(): this.type = js.native
}
