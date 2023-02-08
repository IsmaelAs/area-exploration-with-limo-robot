import { Server } from "socket.io";
//import {AppComponent} from './app/app-component'

export class SocketManager {
    private sio: Server;
    constructor(server: Server ){
        this.sio = server; //= new Server(server, { cors: { origin: '*' } });
    }

    handleSockets(): void {
        this.sio.on('connection', (socket) => {
            console.log('connexion au serveur');
            socket.on('avancer', () => {
               console.log("avancer dans socket manager");
           })
            })  

        
    }
}