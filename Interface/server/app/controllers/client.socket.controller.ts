import { io, Socket } from 'socket.io-client';
import { injectable } from 'inversify';

@injectable()
export class ClientSocketController {
    private socket1: Socket;
    private socket2: Socket;

    constructor(){
        this.socket1 = io('http://localhost:3020');
        this.socket2 = io('http://localhost:3030');
    }

    connectToServer(){
        this.socket1.on('connect', ()=> {

            //receive a message form the server
            this.socket1.on('server-message', (message) => {
                console.log('le client Rover a recu un message du serveur: ')
                console.log(message)

                // send a message back to server
                setTimeout(() => {
                    this.socket1.emit('client-message', 'Ceci est un message du client - Rover')
                }, 1000);
            })
        })

        this.socket2.on('connect', ()=> {

            //receive a message form the server
            this.socket2.on('server-message', (message) => {
                console.log('le client Drone a recu un message du serveur: ')
                console.log(message)

                // send a message back to server
                setTimeout(() => {
                    this.socket2.emit('client-message', 'Ceci est un message du client - Drone')
                }, 1000);
            })
        })
    }
}