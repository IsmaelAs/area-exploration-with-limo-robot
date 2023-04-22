import NodeScan from '../classes/ros/nodes/node-scan';
import { NodePosition } from '../classes/ros/nodes/node-position';
import { Subject } from 'rxjs';
import LogType from '@app/types/LogType';
export class Logger {
  private intervalLog: NodeJS.Timer;

  private nodePosition: NodePosition = new NodePosition();

  private nodeScan: NodeScan = new NodeScan();

  private limoId: number;

  private logsObservable: Subject<LogType> = new Subject();


  startLogs() {
    this.nodePosition.initNodePosition();
    this.nodeScan.initNodeScan();
    this.intervalLog = setInterval(this.callBack.bind(this), 1000);
  }

  private callBack() {
    const position = this.positionLog();
    const scanOutput = this.nodeScan.getData();


    this.logsObservable.next({
      limoId: this.limoId,
      data: {
        position,
        scanOutput,
      },
    });
  }

  private positionLog() {
    const data = this.nodePosition.getData();
    if (!data) return null;

    const distanceFromInit = Math.sqrt((data.pose.pose.position.x ** 2) + (data.pose.pose.position.y ** 2) + (data.pose.pose.position.z ** 2));
    return {
      x: Math.round(data.pose.pose.position.x * 100) / 100,
      y: Math.round(data.pose.pose.position.y * 100) / 100,
      z: Math.round(data.pose.pose.position.z * 100) / 100,
      distanceFromInit: Math.round(distanceFromInit * 100) / 100,
    };
  }

  stopLog() {
    this.logsObservable.next({
      limoId: this.limoId,
      data: `Stop sending logs from limo ${this.limoId}`,
    });

    this.nodePosition.closeNodePosition();
    this.nodeScan.closeNodeScan();
    clearInterval(this.intervalLog);
  }

  setLimoId(limoId: number) {
    this.limoId = limoId;
  }

  get logObservable() {
    return this.logsObservable.asObservable();
  }
}
