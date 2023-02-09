import { Component } from '@angular/core';
import { SocketCommunicationService } from './services/socket-communication/socket-communication.service';
import RobotTargetType from './types/RobotType';

@Component({
  selector: 'app-root',
  templateUrl: './app.component.html',
  styleUrls: ['./app.component.scss']
})
export class AppComponent {

  list: RobotTargetType[] = ["limo-1", "limo-2", "robots"];
  type: RobotTargetType = "limo-1";
  
  constructor(
    private socketCommunication : SocketCommunicationService
    ){}
  

  identify(){
    this.socketCommunication.identify(this.type);
  }

  startMission() {
    this.socketCommunication.startMission(this.type)
  }

  stopMission() {
    this.socketCommunication.stopMission(this.type)
  }


  setType(choice: RobotTargetType) {
    this.type = choice;
  }

}

