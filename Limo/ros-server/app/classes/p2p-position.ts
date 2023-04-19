import { NodePosition } from './ros/nodes/node-position';


export class P2PPosition {
  private nodePosition: NodePosition = new NodePosition();

  private limoId: number;

  private p2pDistance = 0;

  private distance = 0;

  private furthestLimo= 0;


  constructor(limoId: number) {
    this.limoId = limoId;
    if (process.env.IS_SIMULATION) this.nodePosition.setNamespace(`limo${this.limoId}`);
    this.nodePosition.initNodePosition();
    setInterval(this.getFurthestLimo, 2000);
  }


  getDistance() {
    const data = this.nodePosition.getData();1
    

    if (!data) return null;
    const distanceFromInit = Math.sqrt((data.pose.pose.position.x ** 2) + (data.pose.pose.position.y ** 2) + (data.pose.pose.position.z ** 2));
    this.setDistance(Math.round(distanceFromInit * 100) / 100);
    return this.distance;
  }

  stopP2PPosition() {
    this.nodePosition.closeNodePosition();
  }


  private setDistance(distance:number) {
    this.distance = distance;
  }

  getP2PDistance() {
    return this.p2pDistance;
  }

  setP2PDistance(distance:number) {
    this.p2pDistance = distance;
  }

  getFurthestLimo() {
    if (this.distance > this.p2pDistance) this.furthestLimo = this.limoId;
    else this.furthestLimo = this.limoId === 1 ? 2 : 1;
    return this.furthestLimo;
  }
}
