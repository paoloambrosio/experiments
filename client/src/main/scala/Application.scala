import scala.scalajs.js
import org.singlespaced.d3js.d3
import net.paoloambrosio.d3js.cloud._
import scala.util.Random

object Application extends js.JSApp {

  private val width = 960
  private val height = 500

  def main(): Unit = {
    val cloud = d3.layout.cloud()
      .size(js.Array(width, height))
      .words(js.Array(new Word("A", 10), new Word("B", 20), new Word("C", 20)))
      .padding(5)
      .fontSize((w: Word) => w.size)
      .rotate((w: Word) => Random.nextInt(2) * 90)
      .font("Impact")
      .on("end", draw)
    cloud.start()
  }

  private val draw: js.Array[WordTag] => Unit = words => {
    d3.select("body").append("svg")
        .attr("width", width)
        .attr("height", height)
      .append("g")
        .attr("transform", s"translate(${width/2},${height/2})")
      .selectAll("text")
        .data(words)
      .enter().append("text")
//        .style("font-size", (w: WordTag) => "${w.size}px")
        .style("font-family", "Impact")
        .attr("text-anchor", "middle")
//        .attr("transform", (w: WordTag, _: Int) => s"translate(${w.x} ${w.y})rotate(${w.rotate}")
//        .text((w: WordTag, _: Int, _: Int) => w.text)
    // NOTHING WORKS!!!!!!!!!
  }
}
