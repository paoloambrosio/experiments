import akka.cluster.Cluster
import akka.cluster.ClusterEvent._
import akka.actor.ActorLogging
import akka.actor.Actor

class ClusterListenerActor extends Actor {

  val cluster = Cluster(context.system)

  override def preStart(): Unit = {
    cluster.subscribe(self, InitialStateAsEvents, classOf[MemberEvent], classOf[UnreachableMember])
  }

  override def postStop(): Unit = {
    cluster.unsubscribe(self)
  }

  def receive = {
    case MemberUp(member) =>                      colouredLog(s"Up          => ${member.address}")
    case UnreachableMember(member) =>             colouredLog(s"Unreachable => ${member.address}")
    case MemberRemoved(member, previousStatus) => colouredLog(s"Removed     => ${member.address}")
    case _: MemberEvent => // ignore every other message
  }

  private def colouredLog(s: String) = {
    Console.out.println(Console.CYAN_B + Console.BLACK + s + Console.RESET)
  }
}
