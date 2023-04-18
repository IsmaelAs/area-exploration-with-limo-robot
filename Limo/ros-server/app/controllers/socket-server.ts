import { Logger } from '../services/logger';
import { NodeManager } from '../classes/nodes-manager';
import { Server } from 'socket.io';
import delay from 'delay';
import { Subscription } from 'rxjs';
import LogType from '../types/LogType';
import { MyStateMachine } from '../classes/state-machine';
import StateType from '../types/StateType';
import { MissionDistance } from '../services/mission-distance';


const NO_CLIENT = 0;

export class SocketServer {
  private server: Server;

  private nodeManager: NodeManager;

  private clientCounter = NO_CLIENT;

  private logger: Logger;

  private loggerObservable: Subscription;

  private stateMachine: MyStateMachine;

  private isMissionStopped = true;

  limoId: number;

  private missionDistance: MissionDistance;

  constructor(server: Server, nodeManager: NodeManager, logger: Logger) {
    this.server = server;
    this.nodeManager = nodeManager;
    this.logger = logger;
    this.stateMachine = new MyStateMachine();
    this.missionDistance = new MissionDistance(this.server);
  }

  // Connect the socket to the limo node server ; subscribe to all limo command ; start all nodes
  connectSocketServer(): void {
    this.nodeManager.startNodes();
    console.log('normalement je me suis connecté à toutes les nodes');
    // this.emit('test-emit', 'Hello World envoyé directement');
    // this.missionDistance.sendTestMessage('Hello world');

    this.server.on('connection', (socket) => {
      console.log('Connected to node server');

      this.clientCounter++;

      socket.on('login', (limoId: number) => {
        this.limoId = limoId;
        this.logger.setLimoId(this.limoId);
        this.stateMachine.setLimoId(this.limoId);
        this.missionDistance.setLimoId(this.limoId);
        this.nodeManager['nodeBattery'].getBatteryObservable().subscribe(this.sendBattery.bind(this));


        this.stateMachine.stateObservable.subscribe(this.sendState.bind(this));
        this.stateMachine.startStates();
        this.stateMachine.onReady();
      });

      socket.on('identify', async () => {
        await this.nodeManager.identify();
      });

      socket.on('error', (err: Error) => {
        console.log(`On Limo Error : ${err.stack}`);
      });

      socket.on('start-mission', async () => {
        if (!this.isMissionStopped) return;

        this.loggerObservable = this.logger.logObservable.subscribe(this.sendLogs.bind(this));

        this.logger.startLogs();
        this.missionDistance.startMission();
        this.stateMachine.onMission();
        this.isMissionStopped = false;

        // eslint-disable-next-line no-magic-numbers
        await delay(1000);
        this.nodeManager.startMission();
      });

      socket.on('stop-mission', () => {
        if (this.isMissionStopped) return;

        this.isMissionStopped = true;
        this.stateMachine.onMissionEnd();
        this.nodeManager.stopMission();
        this.logger.stopLog();
        this.missionDistance.stopMission();
        this.loggerObservable.unsubscribe();
        this.stateMachine.onReady();
      });

      socket.on('disconnect', () => {
        console.log(`Server disconnected from Limo ${this.limoId} robot`);
        this.clientCounter--;
      });
    });
  }

  emit<T>(event: string, data?: T) {
    data ? this.server.emit(event, data) : this.server.emit(event);
  }

  private sendLogs(log: LogType) {
    if (this.clientCounter > NO_CLIENT) this.emit('save-log', log);
  }

  private sendState(data: StateType) {
    console.log('J\'EMITE MTN L\'ETAT');
    console.log(data);
    this.emit('save-state', data);
  }

  private sendBattery(data: { percentage: number }) {
    this.emit('save-battery', {limoId: this.limoId,
      battery: data.percentage});
  }
}

