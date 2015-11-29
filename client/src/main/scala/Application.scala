import org.scalajs.dom
import org.scalajs.dom.MessageEvent

import scala.scalajs.js
import org.singlespaced.d3js.d3
import net.paoloambrosio.d3js.cloud._
import scala.scalajs.js.JSON
import scala.util.Random
import scala.concurrent.duration._

object Application extends js.JSApp {

  private val width = 960
  private val height = 500
  private val transitionTime = 1 second

  private lazy val cloud = {
    d3.layout.cloud()
      .size(js.Array(width, height))
      .padding(5)
      .fontSize((w: Word) => w.size)
      .rotate((w: Word) => Random.nextInt(2) * 90)
      .font("Impact")
      .on("end", draw)
  }

  private lazy val svg = {
    d3.select("body").append("svg")
      .attr("width", width)
      .attr("height", height)
      .append("g")
      .attr("transform", s"translate(${width/2},${height/2})")
  }

  def main(): Unit = {
    val ws = new dom.WebSocket(s"ws://${dom.document.location.host}/words")
    ws.onmessage = (me: MessageEvent) => {
      val words = JSON.parse(me.data.asInstanceOf[String]).asInstanceOf[js.Array[Word]]
      updateCloud(words)
    }
  }

  def updateCloud(words: js.Array[Word]): Unit = {
    cloud.stop().words(words).start()
  }

  private val draw: js.Array[WordTag] => Unit = words => {
    import org.singlespaced.d3js.Ops._

    val maxSize = words.map(w => w.size).max
    val duration = transitionTime.toMillis.toDouble

    val tags = svg.selectAll("text")
      .data(words)
      .text((w: WordTag) => w.text)

    tags.enter()
      .append("text")
      .text((w: WordTag) => w.text)
      .attr("text-anchor", "middle")
      .style("font-family", "Impact")

    tags.transition().duration(duration)
      .style("font-size", (w: WordTag) => s"${w.size}px")
      .style("opacity", (w: WordTag) => 0.2+(0.8*w.size/maxSize))
      .attr("transform", (w: WordTag) => s"translate(${w.x} ${w.y})rotate(${w.rotate})")

// IT DOES NOT WORK, BUT WE WILL NEVER REMOVE A TAG
//    tags.exit().transition().duration(duration)
//      .style("opacity", 0)
//      .remove()
  }

}
