import { NodePosition } from './ros/nodes/node-position';


export class P2PPosition {
  private nodePosition: NodePosition = new NodePosition();

  private limoId: number;

  private p2pDistance = 0;

  private distance = 0;

  private furthestLimo = 0;

  private interval: NodeJS.Timer;



  constructor(limoID: number) {
    this.limoId = limoID;
    if (process.env.IS_SIMULATION) this.nodePosition.setNamespace(`limo${this.limoId}`);
    this.nodePosition.initNodePosition();
    this.interval = setInterval(this.getFurthestLimo.bind(this), 2000);
  }

  closeInterval(){
    clearInterval(this.interval);
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
    console.log('This is limoId : ' + this.limoId )
    console.log('My distance is : ' + this.distance )
    console.log('The P2P distance is : ' + this.p2pDistance )


    if (this.distance > this.p2pDistance) this.furthestLimo = this.limoId;
    // Return the opposite ID
    else  {
      this.furthestLimo = this.limoId === 1 ? 2 : 1; 
    }
    console.log("the fursthest limo is of ID :  "+ this.furthestLimo)
    return this.furthestLimo;
  }
}
