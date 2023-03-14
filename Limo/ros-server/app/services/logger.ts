/* eslint-disable no-magic-numbers */
import NodeScan from '@app/classes/ros/nodes/node-scan';
import { NodePosition } from '../classes/ros/nodes/node-position';
import { SocketServer } from '../controllers/socket-server';

export class Logger {
  private socketServer: SocketServer;

  private intervalLog: NodeJS.Timer;

  private nodePosition: NodePosition = new NodePosition();

  private nodeScan: NodeScan = new NodeScan();

  constructor(socketServer: SocketServer) {
    this.socketServer = socketServer;
    this.nodePosition.initNodePosition();
    this.nodeScan.initNodeScan();
  }

  startLogs() {
    this.intervalLog = setInterval(this.callBack.bind(this), 1000);
  }

  private callBack() {
    if (this.socketServer.numberSocketConnected === 0) return;

    const position = this.positionLog();
    const scanOutput = this.nodeScan.getData();
    const {limoId} = this.socketServer;
    this.socketServer.emit('save-log', {limoId,
      data: {
        position,
        scanOutput,
      }});
  }

  private positionLog() {
    const data = this.nodePosition.getData();
    const distanceFromInit = Math.sqrt((data.pose.pose.position.x ** 2) + (data.pose.pose.position.y ** 2) + (data.pose.pose.position.z ** 2));
    return {
      x: Math.round(data.pose.pose.position.x * 100) / 100,
      y: Math.round(data.pose.pose.position.y * 100) / 100,
      z: Math.round(data.pose.pose.position.z * 100) / 100,
      distanceFromInit: Math.round(distanceFromInit * 100) / 100,
    };
  }

  stopLog() {
    this.socketServer.emit('save-log', {limoId: this.socketServer.limoId,
      data: 'Stop sending logs'});
    clearInterval(this.intervalLog);
  }
}
