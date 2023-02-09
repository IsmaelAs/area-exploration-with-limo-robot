import { Injectable } from '@angular/core';
import { io, Socket } from 'socket.io-client';


@Injectable({
  providedIn: 'root'
})
export class SocketCommunicationService {

  socket: Socket;
  constructor() {
    this.socket = io('http://localhost:9330');
  }

  avancer(){
    console.log("avancer socket comunication service");
    this.socket.emit('avancer');
  }
}
