import { NodePosition } from './ros/nodes/node-position';


export class P2PPosition {
  private nodePosition: NodePosition = new NodePosition();

  private limoId: number;

  private p2pDistance = 0;

  private distance = 0;


  constructor(limoId: number) {
    this.limoId = limoId;
    if (process.env.IS_SIMULATION) this.nodePosition.setNamespace(`limo${this.limoId}`);
    this.nodePosition.initNodePosition();
  }


  getDistance() {
    const data = this.nodePosition.getData();

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
    if (this.distance > this.p2pDistance) return this.limoId;
    // Return the opposite ID
    return this.limoId === 1 ? 2 : 1;
  }
}
