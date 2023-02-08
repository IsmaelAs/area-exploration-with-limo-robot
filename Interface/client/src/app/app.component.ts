import { Component, OnDestroy, OnInit } from '@angular/core';
import { SocketCommunicationService } from './services/socket-communication/socket-communication.service';

@Component({
  selector: 'app-root',
  templateUrl: './app.component.html',
  styleUrls: ['./app.component.scss']
})
export class AppComponent {
  title = 'client';
  constructor(
    private socketCommunication : SocketCommunicationService){}
  

  avancer(){
    console.log("avanceeeer dans app");
    this.socketCommunication.avancer();
  }

}
