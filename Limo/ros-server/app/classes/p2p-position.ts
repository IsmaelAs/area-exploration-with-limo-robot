import { NodePosition } from './ros/nodes/node-position';


export class P2PPosition {
  private nodePosition: NodePosition = new NodePosition();

  private limoId: number;

  private p2pDistance = 0;


  constructor(limoId: number) {
    this.limoId = limoId;
    if (process.env.IS_SIMULATION) this.nodePosition.setNamespace(`limo${this.limoId}`);
    this.nodePosition.initNodePosition();
  }


  getDistance() {
    const data = this.nodePosition.getData();

    if (!data) return null;
    const distanceFromInit = Math.sqrt((data.pose.pose.position.x ** 2) + (data.pose.pose.position.y ** 2) + (data.pose.pose.position.z ** 2));
    return Math.round(distanceFromInit * 100) / 100;
  }

  stopP2PPosition() {
    this.nodePosition.closeNodePosition();
  }

  getP2PDistance() {
    return this.p2pDistance;
  }

  setP2PDistance(diatance:number) {
    this.p2pDistance = diatance;
  }
}
