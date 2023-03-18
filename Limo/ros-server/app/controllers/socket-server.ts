import { Logger } from '../services/logger';
import { NodeManager } from '../classes/nodes-manager';
import { Server } from 'socket.io';
import delay from 'delay';
import { Subscription } from 'rxjs';
import LogType from '../types/LogType';
import { MyStateMachine } from '../classes/state-machine';
import StateType from '@app/types/StateType';

const NO_CLIENT = 0;

export class SocketServer {
  private server: Server;

  private nodeManager: NodeManager;

  private clientCounter = NO_CLIENT;

  private logger: Logger;

  private loggerObservable: Subscription;



  private stateMachine: MyStateMachine; 

  limoId: number;

  constructor(server: Server) {
    this.server = server;
    this.nodeManager = new NodeManager();
    this.logger = new Logger();
    this.stateMachine = new MyStateMachine();
    this.stateMachine.stateObservable.subscribe(this.sendState.bind(this));
    this.stateMachine.startStates();
  }

  // Connect the socket to the limo node server ; subscribe to all limo command ; start all nodes
  connectSocketServer(): void {
    this.server.on('connection', (socket) => {
      console.log('Connected to node server');
      this.stateMachine.onReady();

      this.clientCounter++;
      socket.on('login', (limoId: number) => {
        this.limoId = limoId;
        this.logger.setLimoId(this.limoId);
        this.stateMachine.setLimoId(this.limoId);
      });

      socket.on('identify', async () => {
        await this.nodeManager.identify();
      });

      socket.on('error', (err: Error) => {
        console.log(`On Limo Error : ${err.stack}`);
      });

      socket.on('start-mission', async () => {
        this.loggerObservable = this.logger.logObservable.subscribe(this.sendLogs.bind(this));

        this.logger.startLogs();
        this.stateMachine.onMission();

        // eslint-disable-next-line no-magic-numbers
        await delay(1000);
        await this.nodeManager.move('forward');
      });

      socket.on('stop-mission', async () => {
        this.stateMachine.onMissionEnd();
        await this.nodeManager.move('backward');
        this.logger.stopLog();
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
    console.log("J'EMITE MTN L'ETAT")
    console.log(data)
    this.emit('save-state', data);
  }
}
