import { Logger } from '../services/logger';
import { NodeManager } from '../classes/nodes-manager';
import { Server } from 'socket.io';
import delay from 'delay';
import { Subscription } from 'rxjs';
import LogType from '@app/types/LogType';
import { NodeExplorationState } from '../classes/ros/nodes/node-exploration-state';

const NO_CLIENT = 0;

export class SocketServer {
  private server: Server;

  private nodeManager: NodeManager;

  private clientCounter = NO_CLIENT;

  private logger: Logger;

  private loggerObservable: Subscription;

  limoId: number;

  constructor(server: Server, nodeExplorationState: NodeExplorationState) {
    this.server = server;
    this.nodeManager = new NodeManager(nodeExplorationState);
    this.logger = new Logger();
  }

  // Connect the socket to the limo node server ; subscribe to all limo command ; start all nodes
  connectSocketServer(): void {
    this.server.on('connection', (socket) => {
      console.log('Connected to node server');

      this.clientCounter++;
      socket.on('login', (limoId: number) => {
        this.limoId = limoId;
        this.logger.setLimoId(this.limoId);
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

        // eslint-disable-next-line no-magic-numbers
        await delay(1000);
        this.nodeManager.startMission();
      });

      socket.on('stop-mission', () => {
        this.nodeManager.stopMission();
        this.logger.stopLog();
        this.loggerObservable.unsubscribe();
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
}
