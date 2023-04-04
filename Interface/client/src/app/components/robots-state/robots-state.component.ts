import { Component, OnDestroy } from '@angular/core';
import { Subscription } from 'rxjs';
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

    private stateSub: Subscription;

    constructor (private socketCommunication : SocketCommunicationService) {
        this.stateSub = this.socketCommunication.subscribeState.subscribe(this.changeState.bind(this));

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
}
