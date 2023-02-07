import { ConnectionService } from '@app/services/connection.service';
import { TYPES } from '@app/types';
import { inject, injectable } from 'inversify';
const io = require('socket.io-client');
@injectable()
export class ClientController {
    socket = io('http://localhost:3000');
    constructor(@inject(TYPES.ConnectionService) private connectionService: ConnectionService) {
        this.configureRouter();
    }
    private configureRouter(): void {
        this.socket.on('connect', () => {
            console.log('connected to server');
          });
          
          this.socket.on('disconnect', () => {
            console.log('disconnected from server');
          });
          
          this.socket.on('message', (message) => {
            console.log('received message: ', message);
          });
          
          // send a message to the server
          this.socket.emit('message', 'hello server');
    }
}
