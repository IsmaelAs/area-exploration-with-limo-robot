import { Component, Input } from '@angular/core';
import { SocketCommunicationService } from 'src/app/services/socket-communication/socket-communication.service';
import RobotTargetType from 'src/app/types/RobotType';

@Component({
    'selector': 'app-action-buttons',
    'templateUrl': './action-buttons.component.html',
    'styleUrls': ['./action-buttons.component.scss']
})
export class ActionButtonsComponent {

  @Input() robotTarget: RobotTargetType = 'limo-1';

  constructor (private socketCommunication: SocketCommunicationService) {}

  identify () {

      this.socketCommunication.identify(this.robotTarget);

  }

  startMission () {

      this.socketCommunication.startMission(this.robotTarget);

  }

  stopMission () {

      this.socketCommunication.stopMission(this.robotTarget);

  }

}
