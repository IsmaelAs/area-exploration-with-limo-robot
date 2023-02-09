import { Server as SocketServer } from 'socket.io';



export class ServerSocketController {
    private io: SocketServer;
   // private port: number;

    constructor(io: SocketServer) {
        this.io = io;
      //  this.port = 3010;
    }

    init() {
        //this.io.listen(this.port);
        this.io.on('connection', (socket) => {
            console.log('Le server a recu une connection du socket:')

            //receive a message from client
            socket.on('client-message', (message) => {
                console.log('Le serveur a recu un message du client')
                console.log(message)
            }) 

            socket.on('avancer', () => {
               console.log("avancer dans socket controller");
               this.io.emit('limo-avancer');
           })
     

        })
    }

}