import { Component } from '@angular/core';
import { SocketCommunicationService } from './services/socket-communication/socket-communication.service';
import RobotTargetType from './types/RobotType';

@Component({
    'selector': 'app-root',
    'templateUrl': './app.component.html',
    'styleUrls': ['./app.component.scss']
})
export class AppComponent {

    list: RobotTargetType[] = [
        'limo-1',
        'limo-2',
        'robots'
    ];

    robotTarget: RobotTargetType = 'limo-1';

    constructor (
    // eslint-disable-next-line no-unused-vars
    private socketCommunication : SocketCommunicationService
    ) {


    }


    identify () {

        this.socketCommunication.identify(this.robotTarget);

    }

    startMission () {

        this.socketCommunication.startMission(this.robotTarget);

    }

    stopMission () {

        this.socketCommunication.stopMission(this.robotTarget);

    }


    setTarget (choice: RobotTargetType) {

        this.robotTarget = choice;

    }

}

