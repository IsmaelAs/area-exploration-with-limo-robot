import { Server as SocketServer } from 'socket.io';
import { ClientSocketLimo } from './client.socket.limo';
import { Logger } from '../services/logger';
import RobotTargetType from '../types/RobotType';
import StateLimo from '../interfaces/state-limo';
import delay = require('delay');
import { MissionInfos } from '../services/mission-infos';
import BatteryLimo from '../interfaces/battery-limo';

const FIRST_LIMO = 1;
const SECOND_LIMO = 2;
const NO_CLIENT = 0;

export class ServerSocketController {
  private io: SocketServer;

  private socketLimo?: ClientSocketLimo;

  private socketLimo2?: ClientSocketLimo;

  private logger: Logger;

  private missionInfos: MissionInfos;

  private clientCounter: number;

  private isMissionStarted = false;


  constructor(io: SocketServer, logger: Logger) {
    this.io = io;
    this.logger = logger;
  }

  initializeSocketServer() {
    this.io.on('connection', (socket) => {
      this.clientCounter++;

      socket.on('p2p-start', () => {
        this.sendEventToLimo('robots', 'p2p-start');
      });

      socket.on('identify', (robotTarget: RobotTargetType) => {
        this.sendEventToLimo(robotTarget, 'identify');
      });

      socket.on('start-mission', (robotTarget: RobotTargetType) => {
        if (!this.isMissionStarted) this.startMission();

        this.sendEventToLimo(robotTarget, 'start-mission');
      });

      socket.on('stop-mission', (robotTarget: RobotTargetType) => {
        if (this.isMissionStarted) this.stopMission();

        this.sendEventToLimo(robotTarget, 'stop-mission');
      });

      socket.on('save-log', (data: unknown) => {
        this.logger.saveUserData(data);
      });

      socket.on('get-all-logs', (missionNumber: number) => {
        this.logger.getAllData(missionNumber, socket);
      });

      socket.on('update', async (robotTarget: RobotTargetType) => {
        this.sendEventToLimo(robotTarget, 'update');

        if ((robotTarget === 'limo-1' || robotTarget === 'robots') && this.socketLimo) {
          this.socketLimo.disconnect();
          // eslint-disable-next-line no-magic-numbers
          await delay(5000);
          this.socketLimo.connectClientSocketToLimo();
        }

        if ((robotTarget === 'limo-2' || robotTarget === 'robots') && this.socketLimo2) {
          this.socketLimo2.disconnect();
          // eslint-disable-next-line no-magic-numbers
          await delay(5000);
          this.socketLimo2.connectClientSocketToLimo();
        }
      });

      socket.on('send-limo-ips', (ips: {limo1: string, limo2: string}) => {
        const LIMO1_URL = `ws://${ips.limo1}:9332`;
        const LIMO2_URL = `ws://${ips.limo2}:${process.env.IS_SIMULATION ? '9333' : '9332'}`;

        if (ips.limo1.replace(' ', '') !== '') {
          this.socketLimo = new ClientSocketLimo(FIRST_LIMO, LIMO1_URL, LIMO2_URL);
          this.socketLimo.connectClientSocketToLimo();

          this.socketLimo.subscribeState.subscribe(this.sendStateToClient.bind(this));
          this.socketLimo.subscribeP2PConnected.subscribe(this.sendP2PConnectedToClient.bind(this));
          this.socketLimo.subscribeBattery.subscribe(this.sendBatteryToClient.bind(this));
        }

        if (ips.limo2.replace(' ', '') !== '') {
          this.socketLimo2 = new ClientSocketLimo(SECOND_LIMO, LIMO2_URL, LIMO1_URL);
          this.socketLimo2.connectClientSocketToLimo();

          this.socketLimo2.subscribeState.subscribe(this.sendStateToClient.bind(this));
          this.socketLimo2.subscribeBattery.subscribe(this.sendBatteryToClient.bind(this));
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
    this.missionInfos.onMissionEnd();


    if (this.socketLimo) this.socketLimo.stopMission();
    if (this.socketLimo2) this.socketLimo2.stopMission();
    this.isMissionStarted = false;
  }

  private startMission() {
    this.logger.startMission();
    this.missionInfos = new MissionInfos();
    this.missionInfos.onMissionStart();

    if (this.socketLimo) this.socketLimo.startMission();
    if (this.socketLimo2) this.socketLimo2.startMission();
    this.isMissionStarted = true;
  }

  private sendStateToClient(data: StateLimo) {
    this.sendEventToFrontend('send-state', data);
  }

  private sendP2PConnectedToClient(data: boolean) {
    this.sendEventToFrontend('p2p-connected', data);
  }

  private sendBatteryToClient(data: BatteryLimo) {
    this.sendEventToFrontend('send-battery', data);
  }

  private sendEventToFrontend<T>(event: string, data?: T) {
    data ? this.io.emit(event, data) : this.io.emit(event);
  }
}
