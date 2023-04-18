import * as http from 'http';
import { AddressInfo } from 'net';
import { Application } from './app';
import { Server as SocketServer } from 'socket.io';
import { SocketServer as SocketManager } from './controllers/socket-server';
import { NodeExplorationState } from './classes/ros/nodes/node-exploration-state';
import { NodeMovement } from './classes/ros/nodes/node-movement';
import { NodeManager } from './classes/nodes-manager';
import { Logger } from './services/logger';
import { NodeUpdate } from './classes/ros/nodes/node-update';


export class Server {
  private static port: number = process.env.IS_SIMULATION && process.env.LIMO_ID === '2' ? 9333 : 9332;

  private static readonly appPort: string | number | boolean = Server.normalizePort(this.port);

  private static readonly baseDix: number = 10;

  private server: http.Server;

  private io: SocketServer;

  private socketManager: SocketManager;

  // StateMachine: MyStateMachine;


  // eslint-disable-next-line no-useless-constructor, no-empty-function
  constructor(private readonly application: Application) {}

  private static normalizePort(val: number | string): number | string | boolean {
    const port: number = typeof val === 'string' ?
parseInt(val,
    this.baseDix) :
val;
    if (isNaN(port)) {
      return val;
    // eslint-disable-next-line no-magic-numbers
    } else if (port >= 0) {
      return port;
    }
    return false;
  }

  init(): void {
    this.application.app.set('port',
        Server.appPort);

    this.server = http.createServer(this.application.app);

    this.server.listen(Server.appPort);
    this.server.on('error',
        // eslint-disable-next-line no-undef
        (error: NodeJS.ErrnoException) => this.onError(error));
    this.server.on('listening',
        () => this.onListening());

    this.io = new SocketServer(this.server, {
      cors: {
        origin: '*',
      },
    });
    const nodeExplorationState = new NodeExplorationState();
    const nodeMovement = new NodeMovement();
    const nodeUpdate = new NodeUpdate();
    const nodeManager = new NodeManager(nodeExplorationState, nodeMovement, nodeUpdate);
    const logger = new Logger();
    this.socketManager = new SocketManager(this.io, nodeManager, logger);
    this.socketManager.connectSocketServer();
  }

  // eslint-disable-next-line class-methods-use-this, no-undef
  private onError(error: NodeJS.ErrnoException): void {
    if (error.syscall !== 'listen') {
      throw error;
    }
    const bind: string = typeof Server.appPort === 'string' ?
`Pipe ${Server.appPort}` :
`Port ${Server.appPort}`;
    switch (error.code) {
      case 'EACCES':
        console.error(`${bind} requires elevated privileges`);
        process.exit(1);
        break;
      case 'EADDRINUSE':
        console.error(`${bind} is already in use`);
        process.exit(1);
        break;
      default:
        throw error;
    }
  }

  /**
   * Se produit lorsque le serveur se met à écouter sur le port.
   */
  private onListening(): void {
    const addr = this.server.address() as AddressInfo;
    const bind: string = typeof addr === 'string' ?
`pipe ${addr}` :
`port ${addr.port}`;
    // eslint-disable-next-line no-console
    console.log(`Listening on ${bind}`);
  }
}
