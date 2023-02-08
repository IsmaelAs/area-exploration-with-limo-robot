import { Server as SocketServer } from 'socket.io';



export class ServerSocketController {
    private io: SocketServer

    constructor(io: SocketServer) {
        this.io = io;
    }

    init() {
        this.io.on('connection', (socket) => {
            console.log('Le server a recu une connection du socket:')

            // send a message to client
            setTimeout(() => {
                socket.emit('server-message', 'Ceci est un message du serveur')
            }, 1000);

            //receive a message from client
            socket.on('client-message', (message) => {
                console.log('Le serveur a recu un message du client')
                console.log(message)
            })
        })
    }

}