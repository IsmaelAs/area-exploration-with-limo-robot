import { Server } from 'socket.io';


const NO_CLIENT = 0;

export class P2PSocketServer {
  private server: Server;

  private clientCounter = NO_CLIENT;

  constructor(server: Server) {
    this.server = server;
  }

  startP2PConnection(): void {
    this.server.on('connection', (socket) => {
      console.log('Connected to node server');

      this.clientCounter++;

      socket.on('disconnect', () => {
        console.log(`Limo2 disconnected from P2P`);
        this.clientCounter--;
      });
    });
  }

  emit<T>(event: string, data?: T) {
        data ? this.server.emit(event, data) : this.server.emit(event);
  }
}


