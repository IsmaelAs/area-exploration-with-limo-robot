import { NodeMovement } from "./ros/nodes/node-movement";
import Command from "../types/Command";
export class NodeManager {
  private nodeMovement: NodeMovement;

  constructor() {
    this.nodeMovement = new NodeMovement();
  }

  // Start all nodes
  start(): void {
    console.log(`Starting connection for the nodes`);
    this.nodeMovement.initNodeMovement()
  }

  // Send command to move limo
  async move(command :Command, nbrSendingMsg?: number): Promise<void>{
    nbrSendingMsg ? await this.nodeMovement.move(command, nbrSendingMsg) : await this.nodeMovement.move(command)
  }

  // Stop all nodes
  stop(): void {
    console.log(`Closing connection of nodes !`);
    this.nodeMovement.closeNodeMovement()
  }
}