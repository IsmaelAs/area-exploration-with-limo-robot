import { Component, Input} from '@angular/core';
import { SocketCommunicationService } from './services/socket-communication/socket-communication.service';

@Component({
  selector: 'app-root',
  templateUrl: './app.component.html',
  styleUrls: ['./app.component.scss']
})
export class AppComponent {
  title = 'client';
  private type: string = "";
  liste = ["Drome", "Limo", "Les 2"];
 
  constructor(
    private socketCommunication : SocketCommunicationService){}
  

  avancer(){
    console.log("avanceeeer dans app");
    this.socketCommunication.avancer();
  }
  selectionner(choix: string) {
    this.type = choix;
    console.log(this.type);
  }

}
