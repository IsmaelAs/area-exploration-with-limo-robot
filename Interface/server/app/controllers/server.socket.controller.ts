import { Server as SocketServer } from 'socket.io';



export class ServerSocketController {
    private io: SocketServer;

    constructor(io: SocketServer) {
        this.io = io;
    }

    init() {
        this.io.on('connection', (socket) => {
            console.log('Le server a recu une connection du socket:')


            socket.on('avancer', () => {
                console.log("avancer dans socket controller");
                this.io.emit('limo-avancer');
            })


        })
    }

}