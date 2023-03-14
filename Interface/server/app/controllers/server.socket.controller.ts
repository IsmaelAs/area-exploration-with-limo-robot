import { Server as SocketServer } from 'socket.io';
import { ClientSocketLimo } from './client.socket.limo';
import { Logger } from '../services/logger';
import RobotTargetType from '../types/RobotType';

const FIRST_LIMO = 1;
const SECOND_LIMO = 2;
const NO_CLIENT = 0;

export class ServerSocketController {
  private io: SocketServer;

  private socketLimo?: ClientSocketLimo;

  private socketLimo2?: ClientSocketLimo;

  private logger: Logger;

  private clientCounter: number;

  constructor(io: SocketServer) {
    this.io = io;
    this.logger = new Logger();
  }

  initializeSocketServer() {
    this.io.on('connection', (socket) => {
      this.clientCounter++;

      socket.on('identify', (robotTarget: RobotTargetType) => {
        this.sendEventToLimo(robotTarget, 'identify');
      });

      socket.on('start-mission', (robotTarget: RobotTargetType) => {
        this.startMission();
        this.sendEventToLimo(robotTarget, 'start-mission');
      });

      socket.on('stop-mission', (robotTarget: RobotTargetType) => {
        this.stopMission();
        this.sendEventToLimo(robotTarget, 'stop-mission');
      });

      socket.on('save-log', (data: unknown) => {
        this.logger.saveUserData(data);
      });


      socket.on('get-all-logs', (missionNumber: number) => {
        this.logger.getAllData(missionNumber, socket);
      });

      /*
       * Socket.on('save-state', (data: {limoId: number, state: State}) => {
       *   socket.emit('send-all-logs', `L'état du robot ${data.limoId} est: ${data.state}`);
       *   /*
       *    client --> save-state --> server
       *    server --> send-all-logs --> client
       *    emit stop =--> stop
       */

      //   */
      // });

      socket.on('send-limo-ips', (ips: {limo1: string, limo2: string}) => {
        if (ips.limo1.replace(' ', '') !== '') {
          this.socketLimo = new ClientSocketLimo(FIRST_LIMO, `ws://${ips.limo1}:${process.env.IS_SIMULATION ? process.env.PORT_LIMO_1 : '9332'}`);
          this.socketLimo.connectClientSocketToLimo();
        }

        if (ips.limo2.replace(' ', '') !== '') {
          this.socketLimo2 = new ClientSocketLimo(SECOND_LIMO, `ws://${ips.limo2}:${process.env.IS_SIMULATION ? process.env.PORT_LIMO_2 : '9332'}`);
          this.socketLimo2.connectClientSocketToLimo();
        }
      });

      socket.on('disconnect', () => {
        this.clientCounter--;

        if (this.clientCounter <= NO_CLIENT) {
          this.sendEventToLimo('robots', 'stop-mission');
          this.socketLimo?.disconnect();
          this.socketLimo2?.disconnect();
        }
      });
    });
  }

  private sendEventToLimo<T>(robotTarget: RobotTargetType, event: string, data?: T) {
    if ((robotTarget === 'limo-1' || robotTarget === 'robots') && this.socketLimo) {
      data ? this.socketLimo.emitToLimo(event, data) : this.socketLimo.emitToLimo(event);
    }

    if ((robotTarget === 'limo-2' || robotTarget === 'robots') && this.socketLimo2) {
      data ? this.socketLimo2.emitToLimo(event, data) : this.socketLimo2.emitToLimo(event);
    }
  }

  private stopMission() {
    this.logger.stopMission();

    if (this.socketLimo) this.socketLimo.stopMission();
    if (this.socketLimo2) this.socketLimo2.stopMission();
  }

  private startMission() {
    this.logger.startMission();

    if (this.socketLimo) this.socketLimo.startMission();
    if (this.socketLimo2) this.socketLimo2.startMission();
  }
}
