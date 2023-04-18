import { io, Socket } from 'socket.io-client';
import { Logger } from '../services/logger';
import LogLimo from '../interfaces/log-limo';
import StateLimo from '../interfaces/state-limo';
import { Subject } from 'rxjs';

export class ClientSocketLimo {
  private socket: Socket;

  private logger: Logger;

  private limoId: number;

  private p2pUrl: string;

  private stateObservable: Subject<StateLimo> = new Subject();

  private p2pConnectedObservable: Subject<boolean> = new Subject();


  constructor(limoId: number, limoUrl: string, p2pUrl: string) {
    this.limoId = limoId;
    this.socket = io(limoUrl);
    this.logger = new Logger();
    this.p2pUrl = p2pUrl;
  }

  get subscribeState() {
    return this.stateObservable.asObservable();
  }

  get subscribeP2PConnected() {
    return this.p2pConnectedObservable.asObservable();
  }

  connectClientSocketToLimo() {
    this.socket.on('connect', () => {
      console.log(`Limo ${this.limoId} connected to the ROS server`);
      this.logger.saveLimoData({limoId: this.limoId,
        data: `Limo ${this.limoId} connected to the ROS server`});
      this.socket.emit('login', this.limoId);
      this.socket.emit('p2p-login', this.p2pUrl);
    });

    this.socket.on('reconnect', () => {
      console.log(`Limo ${this.limoId} reconnected to the ROS server`);
      this.logger.saveLimoData({limoId: this.limoId,
        data: `Limo ${this.limoId} connected to the ROS server`});
      this.socket.emit('login', this.limoId);
      this.socket.emit('p2p-login', this.p2pUrl);
    });

    this.socket.on('save-log', (data: LogLimo) => {
      this.logger.saveLimoData(data);
    });

    this.socket.on('save-state', (data: StateLimo) => {
      this.stateObservable.next(data);
    });

    this.socket.on('p2p-connected', (isConnected: boolean) => {
      this.p2pConnectedObservable.next(isConnected);
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
