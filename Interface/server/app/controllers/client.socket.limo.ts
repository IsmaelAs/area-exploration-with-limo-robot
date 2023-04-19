import { io, Socket } from 'socket.io-client';
import { Logger } from '../services/logger';
import LogLimo from '../interfaces/log-limo';
import StateLimo from '../interfaces/state-limo';
import { Subject } from 'rxjs';
import DistanceInfo from '../interfaces/distance-info';
import { MissionInfos } from '../services/mission-infos';
import BatteryLimo from '../interfaces/battery-limo';

export class ClientSocketLimo {
  private socket: Socket;

  private logger: Logger;

  private missionInfos: MissionInfos;

  private limoId: number;

  private p2pUrl: string;


  private stateObservable: Subject<StateLimo> = new Subject();

  private p2pConnectedObservable: Subject<boolean> = new Subject();

  private limoUrl: string;

  private batteryObservable: Subject<BatteryLimo> = new Subject();


  constructor(limoId: number, limoUrl: string, p2pUrl: string, missionInfos: MissionInfos) {
    this.limoId = limoId;
    this.logger = new Logger();
    this.missionInfos = missionInfos;
    this.limoUrl = limoUrl;
    this.p2pUrl = p2pUrl;
  }

  get subscribeState() {
    return this.stateObservable.asObservable();
  }

  get subscribeP2PConnected() {
    return this.p2pConnectedObservable.asObservable();
  }

  get subscribeBattery() {
    return this.batteryObservable.asObservable();
  }

  connectClientSocketToLimo() {
    this.socket = io(this.limoUrl);

    this.socket.on('connect', () => {
      console.log(`Limo ${this.limoId} connected to the ROS server`);
      this.logger.saveLimoData({limoId: this.limoId,
        data: `Limo ${this.limoId} connected to the ROS server`});
      this.socket.emit('login', this.limoId);
      this.socket.emit('p2p-login', this.p2pUrl);
    });

    this.socket.on('save-log', (data: LogLimo) => {
      this.logger.saveLimoData(data);
    });

    this.socket.on('save-total-distance', async (data: DistanceInfo) => {
      console.log('ici jai recu la data dans le emit ');
      console.log(data);
      this.missionInfos.onMissionEnd();
      await this.missionInfos.saveTotalDistance(data);
    });

    this.socket.on('test-emit', (data: string) => {
      console.log('Received data:', data);
    });

    this.socket.on('save-state', (data: StateLimo) => {
      this.stateObservable.next(data);
    });

    this.socket.on('p2p-connected', (isConnected: boolean) => {
      this.p2pConnectedObservable.next(isConnected);
    });

    this.socket.on('save-battery', (data: BatteryLimo) => {
      this.batteryObservable.next(data);
    });

    this.socket.on('disconnect', () => {
      this.socket.removeAllListeners();
    });
  }

  emitToLimo<T>(event: string, data?: T) {
        data ? this.socket.emit(event, data) : this.socket.emit(event);
  }

  startMission() {
    this.logger.startMission();
  }

  stopMission() {
    this.logger.stopMission();
  }

  disconnect() {
    this.socket.disconnect();
  }
}
