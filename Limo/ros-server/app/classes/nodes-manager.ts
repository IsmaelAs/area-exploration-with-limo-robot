import { NodeMouvement } from "./ros/nodes/node-mouvement";

export class NodeManager {
  private a: number;

  private nodeMouvement: NodeMouvement;

  constructor() {
    this.a = 3;
    this.nodeMouvement = new NodeMouvement();
  }

  start() {
    console.log(`Starting connection to node server at ${this.a}`);
  }

  move(command :string){
    console.log(`moving ${command}`);
    this.nodeMouvement.move(command);
  }

  stop() {
    console.log(`Stopping connection to node server at ${this.a}`);
  }
}