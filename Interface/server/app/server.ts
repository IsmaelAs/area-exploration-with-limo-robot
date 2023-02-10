import * as http from 'http';
import { AddressInfo } from 'net';
import { Service } from 'typedi';
import { Application } from './app';
import { Server as SocketServer } from 'socket.io';
import { ServerSocketController } from './controllers/server.socket.controller';
import { ClientSocketLimo1 } from './controllers/client.socket.limo';
import { ClientSocketLimo2 } from './controllers/client.socket.limo2';


@Service()
export class Server {
    private static readonly appPort: string | number | boolean = Server.normalizePort(9330);
    private static readonly baseDix: number = 10;
    private server: http.Server;
    private io: SocketServer;
    private serverSocketController: ServerSocketController;
    private socketLimo?: ClientSocketLimo1
    private socketLimo2?: ClientSocketLimo2


    constructor(private readonly application: Application) {}

    private static normalizePort(val: number | string): number | string | boolean {
        const port: number = typeof val === 'string' ? parseInt(val, this.baseDix) : val;
        if (isNaN(port)) {
            return val;
        } else if (port >= 0) {
            return port;
        } else { 
            return false;
        }
    }

    init(): void {
        this.application.app.set('port', Server.appPort);
        this.server = http.createServer(this.application.app);
        console.log(Server.appPort);
        
        this.server.listen(Server.appPort);
        this.server.on('error', (error: NodeJS.ErrnoException) => this.onError(error));
        this.server.on('listening', () => this.onListening());
        
        console.log("Init socket manager");
        this.io = new SocketServer(this.server, {
            cors: {
                origin: "*"
            }
        })

        if (process.env.LIMO_IP_1) {
            this.socketLimo = new ClientSocketLimo1()
            this.socketLimo.connectClientSocketToLimo1()
        }
        if (process.env.LIMO_IP_2) {
            this.socketLimo2 = new ClientSocketLimo2()
            this.socketLimo2.connectClientSocketToLimo2()
        }

        this.serverSocketController = new ServerSocketController(this.io, this.socketLimo, this.socketLimo2);
        this.serverSocketController.initializeSocketServer();

    }

    private onError(error: NodeJS.ErrnoException): void {
        if (error.syscall !== 'listen') {
            throw error;
        }
        const bind: string = typeof Server.appPort === 'string' ? 'Pipe ' + Server.appPort : 'Port ' + Server.appPort;
        switch (error.code) {
            case 'EACCES':
                // eslint-disable-next-line no-console
                console.error(`${bind} requires elevated privileges`);
                process.exit(1);
                break;
            case 'EADDRINUSE':
                // eslint-disable-next-line no-console
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
        const bind: string = typeof addr === 'string' ? `pipe ${addr}` : `port ${addr.port}`;
        // eslint-disable-next-line no-console
        console.log(`Listening on ${bind}`);
    }
}
