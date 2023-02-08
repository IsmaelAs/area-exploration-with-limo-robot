import { io, Socket } from 'socket.io-client';
import { injectable } from 'inversify';

@injectable()
export class ClientSocketController {
    private socket: Socket

    constructor(){
        this.socket = io('http://localhost:3000')
    }

    connectToServer(){
        this.socket.on('connect', ()=> {

            //receive a message form the server
            this.socket.on('server-message', (message) => {
                console.log('le client a recu un message du serveur')
                console.log(message)

                // send a message back to server
                setTimeout(() => {
                    this.socket.emit('client-message', 'Ceci est un message du client')
                }, 1000);
            })
        })
    }
}