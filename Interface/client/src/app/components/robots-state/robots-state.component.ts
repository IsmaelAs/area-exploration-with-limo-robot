import { Component, OnDestroy } from '@angular/core';
import { Subscription } from 'rxjs';
import { BatteryType } from 'src/app/interfaces/battery-limo';
import { StateType } from 'src/app/interfaces/state-limo';
import { SocketCommunicationService } from 'src/app/services/socket-communication/socket-communication.service';


const FIRST_LIMO = 1;
const SECOND_LIMO = 2;
@Component({
    'selector': 'app-robots-state',
    'templateUrl': './robots-state.component.html',
    'styleUrls': ['./robots-state.component.scss']
})
export class RobotsStateComponent implements OnDestroy {
    stateLimo1 = 'INIT';

    stateLimo2 = 'INIT';

    batteryLimo1: number | string = 'non défini';

    batteryLimo2: number | string = 'non défini';

    private stateSub: Subscription;

    private batterySub: Subscription;

    constructor (private socketCommunication : SocketCommunicationService) {
        this.stateSub = this.socketCommunication.subscribeState.subscribe(this.changeState.bind(this));
        this.batterySub = this.socketCommunication.subscribeBattery.subscribe(this.changeBattery.bind(this));

    }

    ngOnDestroy () {

        this.stateSub.unsubscribe();

    }

    private changeState (newState: StateType) {
        if (newState.limoId === FIRST_LIMO) {
            this.stateLimo1 = newState.state;
        } else if (newState.limoId === SECOND_LIMO) {
            this.stateLimo2 = newState.state;
        }
    }

    private changeBattery (newBattery: BatteryType) {
        if (newBattery.limoId === FIRST_LIMO) {
            this.batteryLimo1 = newBattery.battery;
        } else if (newBattery.limoId === SECOND_LIMO) {
            this.batteryLimo2 = newBattery.battery;
        }
    }
}
