import Command from '../types/types';
import { NodeExplorationState } from './ros/nodes/node-exploration-state';
import { NodeMovement } from './ros/nodes/node-movement';
import { NodeUpdate } from './ros/nodes/node-update';

export class NodeManager {
  private nodeMovement: NodeMovement;

  private nodeExplorationState: NodeExplorationState;

  private nodeUpdate: NodeUpdate;

  constructor(nodeExplorationState: NodeExplorationState, nodeMovement: NodeMovement, nodeUpdate: NodeUpdate) {
    this.nodeMovement = nodeMovement;
    this.nodeExplorationState = nodeExplorationState;
    this.nodeUpdate = nodeUpdate;
  }

  // Start all nodes
  startNodes(): void {
    console.log(`Starting connection for the nodes`);
    this.nodeMovement.initNodeMovement();
    this.nodeExplorationState.initNodeExplorationState();
    this.nodeUpdate.initNodeScan();
  }

  // Send command to move limo
  async move(command :Command, nbrSendingMsg?: number): Promise<void> {
    nbrSendingMsg ? await this.nodeMovement.move(command, nbrSendingMsg) : await this.nodeMovement.move(command);
  }

  async identify(): Promise<void> {
    await this.nodeMovement.move('left-forward');
  }

  startMission() {
    this.nodeExplorationState.sendMessage({ data: true });
  }

  stopMission() {
    this.nodeExplorationState.sendMessage({ data: false });
  }

  async update() {
    await this.nodeMovement.move('forward', 2);
    await this.nodeMovement.move('backward', 2);
    this.nodeUpdate.restartContainers();
  }

  // Stop all nodes
  stop(): void {
    console.log(`Closing connection of nodes !`);
    this.nodeMovement.closeNodeMovement();
    this.nodeExplorationState.closeNodeExplorationState();
    this.nodeUpdate.closeNodeUpdate();
  }
}
