import { io, Socket } from 'socket.io-client';
import { Logger } from '../services/logger';
import LogLimo from '../interfaces/log-limo';
import StateLimo from '@app/interfaces/state-limo';
import { Subject } from 'rxjs';

export class ClientSocketLimo {
  private socket: Socket;

  private logger: Logger;

  private limoId: number;

  private stateObservable: Subject<string> = new Subject();



  constructor(limoId: number, limoUrl: string) {
    this.limoId = limoId;
    this.socket = io(limoUrl);
    this.logger = new Logger();
  }

  get subscribeState (){

    return this.stateObservable.asObservable();
}

  connectClientSocketToLimo() {
    this.socket.on('connect', () => {
      console.log(`Limo ${this.limoId} connected to the ROS server`);
      this.logger.saveLimoData({limoId: this.limoId,
        data: `Limo ${this.limoId} connected to the ROS server`});

      this.socket.on('save-log', (data: LogLimo) => {
        this.logger.saveLimoData(data);
      });

      this.socket.on('save-state', (data: StateLimo) => {
        this.stateObservable.next(data.state);
      });

      this.socket.on('disconnect', () => {
        this.socket.removeAllListeners();
      });

      this.socket.emit('login', this.limoId);
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
