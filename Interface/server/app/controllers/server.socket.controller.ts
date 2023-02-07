import { ConnectionService } from '@app/services/connection.service';
import { TYPES } from '@app/types';
import { inject, injectable } from 'inversify';
import * as sio from 'socket.io';
const io = require('socket.io');
@injectable()
export class ServerController {
    server = io(3000);
    constructor(@inject(TYPES.ConnectionService) private connectionService: ConnectionService) {
        this.configureRouter();
    }
    private configureRouter(): void {
        this.server.on('connection', (socket: sio.Socket) => {
            console.log('a user connected');
            
            socket.on('disconnect', () => {
              console.log('user disconnected');
            });
            
            socket.on('message', (message: String) => {
              console.log('message: ', message);
              socket.emit('message', message);
            });
          });
    }
}
