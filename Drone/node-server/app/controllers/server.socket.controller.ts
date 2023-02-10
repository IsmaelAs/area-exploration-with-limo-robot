import { Server as SocketServer } from 'socket.io';
import { injectable } from 'inversify';

@injectable()
export class ServerSocketController {
    private io: SocketServer;
    private port: number;

    constructor(io: SocketServer) {
        this.io = io;
        this.port = 3030; // or any other port number
    }

    init() {
        this.io.listen(this.port);
        this.io.on('connection', (socket) => {

            // send a message to client
            setTimeout(() => {
                socket.emit('server-message', 'Ceci est un message du serveur Drone')
            }, 1000);

            //receive a message from client
            socket.on('client-message', (message) => {
                console.log('Le serveur a recu un message du client')
                console.log(message)
            })
        })
    }
}