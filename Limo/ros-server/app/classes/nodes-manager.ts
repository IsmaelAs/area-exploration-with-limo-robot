import Command from '../types/types';
import { NodeBattery } from './ros/nodes/node-battery';
import { NodeExplorationState } from './ros/nodes/node-exploration-state';
import { NodeMovement } from './ros/nodes/node-movement';
import { NodeUpdate } from './ros/nodes/node-update';
import { NodeReturnToBase } from './ros/nodes/node-return-to-base';

export class NodeManager {
  private nodeMovement: NodeMovement;

  private nodeExplorationState: NodeExplorationState;

  private nodeReturnToBase: NodeReturnToBase;

  private nodeUpdate: NodeUpdate;

  private nodeBattery: NodeBattery;

  constructor(nodeExplorationState: NodeExplorationState, nodeReturnToBase: NodeReturnToBase,
      nodeMovement: NodeMovement, nodeUpdate: NodeUpdate, nodeBattery: NodeBattery) {
    this.nodeMovement = nodeMovement;
    this.nodeExplorationState = nodeExplorationState;
    this.nodeReturnToBase = nodeReturnToBase;
    this.nodeUpdate = nodeUpdate;
    this.nodeBattery = nodeBattery;
  }

  // Start all nodes
  startNodes(): void {
    console.log(`Starting connection for the nodes`);
    this.nodeMovement.initNodeMovement();
    this.nodeExplorationState.initNodeExplorationState();
    this.nodeReturnToBase.initNodeReturnToBase();
    this.nodeUpdate.initNodeScan();
    this.nodeBattery.initNodeBattery();
  }

  // Send command to move limo
  async move(command: Command, nbrSendingMsg?: number): Promise<void> {
    nbrSendingMsg ? await this.nodeMovement.move(command, nbrSendingMsg) : await this.nodeMovement.move(command);
  }

  async identify(): Promise<void> {
    await this.nodeMovement.move('left-forward');
  }

  startMission() {
    this.nodeExplorationState.sendMessage({ data: true });
  }

  returnToBase() {
    this.nodeReturnToBase.sendMessage({ data: true });
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
    this.nodeBattery.closeNodeBattery();
  }
}
