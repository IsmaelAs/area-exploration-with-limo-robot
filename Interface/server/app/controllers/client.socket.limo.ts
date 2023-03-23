import { io, Socket } from 'socket.io-client';
import { Logger } from '../services/logger';
import LogLimo from '../interfaces/log-limo';
import StateLimo from '../interfaces/state-limo';
import { Subject } from 'rxjs';

export class ClientSocketLimo {
  private socket: Socket;

  private logger: Logger;

  private limoId: number;

  private stateObservable: Subject<StateLimo> = new Subject();


  constructor(limoId: number, limoUrl: string) {
    this.limoId = limoId;
    this.socket = io(limoUrl);
    this.logger = new Logger();
  }

  get subscribeState() {
    return this.stateObservable.asObservable();
  }

  connectClientSocketToLimo() {
    this.socket.on('connect', () => {
      console.log(`Limo ${this.limoId} connected to the ROS server`);
      this.logger.saveLimoData({limoId: this.limoId,
        data: `Limo ${this.limoId} connected to the ROS server`});
      this.socket.emit('login', this.limoId);
    });
    this.socket.on('save-log', (data: LogLimo) => {
      this.logger.saveLimoData(data);
    });

    this.socket.on('save-state', (data: StateLimo) => {
      console.log('ICI JE RECOIS DANS SERVER-INTERFACE MON ETAT');
      console.log(data);
      this.stateObservable.next(data);
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
