import { NodeMouvement } from "./ros/nodes/node-mouvement";
import Command from "../types/Command";
export class NodeManager {
  private nodeMouvement: NodeMouvement;

  constructor() {
    this.nodeMouvement = new NodeMouvement();
  }

  // Start all nodes
  start(): void {
    console.log(`Starting connection for the nodes`);
    this.nodeMouvement.initNodeMouvement()
  }

  // Send command to move limo
  async move(command :Command, nbrSendingMsg?: number): Promise<void>{
    nbrSendingMsg ? await this.nodeMouvement.move(command, nbrSendingMsg) : await this.nodeMouvement.move(command)
  }

  // Stop all nodes
  stop(): void {
    console.log(`Closing connection of nodes !`);
    this.nodeMouvement.closeNodeMouvement()
  }
}