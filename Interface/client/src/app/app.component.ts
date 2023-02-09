import { Component } from '@angular/core';
import { SocketCommunicationService } from './services/socket-communication/socket-communication.service';
import RobotTargetType from './types/RobotType';

@Component({
  selector: 'app-root',
  templateUrl: './app.component.html',
  styleUrls: ['./app.component.scss']
})
export class AppComponent {

  list: RobotTargetType[] = ["drone", "limo", "robots"];
  private type: RobotTargetType = "limo";
  
  constructor(
    private socketCommunication : SocketCommunicationService
    ){}
  

  advance(){
    this.socketCommunication.advance(this.type);
  }

  startMission() {
    this.socketCommunication.startMission(this.type)
  }

  stopMission() {
    this.socketCommunication.stopMission(this.type)
  }


  setChoice(choice: RobotTargetType) {
    this.type = choice;
  }

}

