import { NodePosition } from '../classes/ros/nodes/node-position';
import { Server } from 'socket.io';

export class MissionDistance {
  private nodePosition: NodePosition = new NodePosition();

  private previousPosition = { x: 0,
    y: 0,
    z: 0 };

  private totalDistance = 0;

  private isMissionActive = false;

  private limoId: number;

  private server: Server;

  private updateInterval: NodeJS.Timer;

  constructor(server: Server) {
    this.server = server;
  }

  startMission(): void {
    this.isMissionActive = true;
    this.nodePosition.initNodePosition();
    this.previousPosition = this.getCurrentPosition();
    this.updateInterval = setInterval(() => this.calculateDistance(), 1000);
  }

  stopMission(): void {
    if (!this.isMissionActive) return;

    this.isMissionActive = false;
    clearInterval(this.updateInterval);
    this.nodePosition.closeNodePosition();
    this.emitTotalDistance();
    this.totalDistance = 0;
  }

  setLimoId(limoId: number): void {
    this.limoId = limoId;
  }

  private getCurrentPosition() {
    const data = this.nodePosition.getData();

    return {
      x: data.pose.pose.position.x,
      y: data.pose.pose.position.y,
      z: data.pose.pose.position.z,
    };
  }

  private calculateDistance(): void {
    if (!this.isMissionActive) return;

    const currentPosition = this.getCurrentPosition();
    const distance = Math.sqrt(
        ((currentPosition.x - this.previousPosition.x) ** 2) +
        ((currentPosition.y - this.previousPosition.y) ** 2) +
        ((currentPosition.z - this.previousPosition.z) ** 2)
    );

    this.totalDistance += distance;
    this.previousPosition = currentPosition;
  }

  private emitTotalDistance(): void {
    const id = this.limoId;
    const distance = Math.round(this.totalDistance * 100) / 100;
    console.log('la distance parcourue est: ');
    console.log(distance);
    console.log('pour le limo ');
    console.log(id);
    this.server.emit('save-total-distance', {
      limoId: this.limoId,
      totalDistance: Math.round(this.totalDistance * 100) / 100,
    });
  }
}
