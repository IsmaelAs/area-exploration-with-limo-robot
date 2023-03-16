import Command from '../types/Command';
import { NodeMovement } from './ros/nodes/node-movement';

export class NodeManager {
  private nodeMovement: NodeMovement;

  constructor() {
    this.nodeMovement = new NodeMovement();
    this.startNodes();
  }

  // Start all nodes
  startNodes(): void {
    console.log(`Starting connection for the nodes`);
    this.nodeMovement.initNodeMovement();
  }

  // Send command to move limo
  async move(command :Command, nbrSendingMsg?: number): Promise<void> {
    nbrSendingMsg ? await this.nodeMovement.move(command, nbrSendingMsg) : await this.nodeMovement.move(command);
  }

  async identify(): Promise<void> {
    await this.nodeMovement.move('left-forward');
  }

  // Stop all nodes
  stop(): void {
    console.log(`Closing connection of nodes !`);
    this.nodeMovement.closeNodeMovement();
  }
}
