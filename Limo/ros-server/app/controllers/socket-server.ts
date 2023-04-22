import { Logger } from '../services/logger';
import { NodeManager } from '../classes/nodes-manager';
import { Server } from 'socket.io';
import { Subscription } from 'rxjs';
import LogType from '../types/LogType';
import { MyStateMachine } from '../classes/state-machine';
import StateType from '../types/StateType';
import { P2PSocketClient } from './p2p-socket-client';
import { P2PPosition } from '../classes/p2p-position';
import Map from '../types/Map';
import NodeMap from '../classes/ros/nodes/node-map';
import { MissionDistance } from '../services/mission-distance';


const NO_CLIENT = 0;

export class SocketServer {
  private server: Server;

  private nodeManager: NodeManager;

  private p2pSocketClient?: P2PSocketClient;

  private clientCounter = NO_CLIENT;

  private logger: Logger;

  private loggerObservable: Subscription;

  private stateMachine: MyStateMachine;

  private isMissionStopped = true;

  private p2pUrl = '';

  private p2pPosition: P2PPosition;

  private intervalPos: NodeJS.Timer;

  private nodeMap: NodeMap = new NodeMap();

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

    this.server.on('connection', (socket) => {
      console.log('Connected to node server');

      this.clientCounter++;

      socket.on('login', (limoId: number) => {
        console.log('login from', limoId);
        this.limoId = limoId;
        this.logger.setLimoId(this.limoId);
        this.stateMachine.setLimoId(this.limoId);
        this.missionDistance.setLimoId(this.limoId);
        this.nodeManager['nodeBattery'].getBatteryObservable().subscribe(this.sendBattery.bind(this));


        this.stateMachine.stateObservable.subscribe(this.sendState.bind(this));
        this.stateMachine.startStates();
        this.stateMachine.onReady();
      });

      socket.on('p2p-login', (p2pUrl: string) => {
        this.p2pUrl = p2pUrl;
        if (this.limoId === 2) this.p2pSocketClient = new P2PSocketClient(this.p2pUrl);
        else this.p2pPosition = new P2PPosition(1);
      });

      socket.on('p2p-start', () => {
        if (this.limoId === 2) this.p2pSocketClient?.activateP2P();
        else {
          this.p2pPosition.activateP2P();
          this.nodeMap.initNodeMap();
          this.intervalPos = setInterval(this.callBackPos.bind(this), 1000);
        }
      });

      socket.on('p2p-stop', () => {
        if (this.limoId === 2) this.p2pSocketClient?.deactivateP2P();
        else clearInterval(this.intervalPos);
      });

      socket.on('p2p-activated', () => {
        if (this.limoId === 2) return;
        socket.broadcast.emit('p2p-connected', true);
      });

      socket.on('p2p-deactivated', () => {
        if (this.limoId === 2) return;
        socket.broadcast.emit('p2p-connected', false);
      });

      socket.on('p2p-distance', (distance: number) => {
        if (this.limoId === 2) return;
        this.p2pPosition.setP2PDistance(distance);
      });

      socket.on('p2p-map', (map: Map) => {
        if (this.limoId === 2) return;
        this.nodeMap.sendMap(map);
      });

      socket.on('identify', async () => {
        await this.nodeManager.identify();
      });

      socket.on('error', (err: Error) => {
        console.log(`On Limo Error : ${err.stack}`);
      });

      // eslint-disable-next-line require-await
      socket.on('start-mission', async () => {
        if (!this.isMissionStopped) return;

        this.loggerObservable = this.logger.logObservable.subscribe(this.sendLogs.bind(this));

        this.logger.startLogs();
        this.stateMachine.onMission();
        this.isMissionStopped = false;
        this.nodeManager.startMission();
      });

      // eslint-disable-next-line require-await
      socket.on('return-to-base', async () => {
        this.nodeManager.returnToBase();
      });

      socket.on('stop-mission', () => {
        if (this.isMissionStopped) return;

        this.isMissionStopped = true;
        this.stateMachine.onMissionEnd();
        this.nodeManager.stopMission();
        this.logger.stopLog();
        this.loggerObservable.unsubscribe();
        this.stateMachine.onReady();
      });

      socket.on('update', async () => {
        await this.nodeManager.update();
      });

      socket.on('disconnect', () => {
        console.log(`Server disconnected from Limo ${this.limoId} robot`);
        this.clientCounter--;
      });
    });
  }

  private callBackPos() {
    const distance = this.p2pPosition.getDistance();
    if (distance) this.emit('p2p-distance', distance);
  }

  emit<T>(event: string, data?: T) {
    data ? this.server.emit(event, data) : this.server.emit(event);
  }

  private sendLogs(log: LogType) {
    if (this.clientCounter > NO_CLIENT) this.emit('save-log', log);
  }

  private sendState(data: StateType) {
    this.emit('save-state', data);
  }

  private sendBattery(data: { percentage: number }) {
    this.emit('save-battery', {
      limoId: this.limoId,
      battery: data.percentage,
    });
  }
}

