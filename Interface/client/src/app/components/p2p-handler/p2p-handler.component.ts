import { Component, OnDestroy } from '@angular/core';
import { Subscription } from 'rxjs';
import { SocketCommunicationService } from 'src/app/services/socket-communication/socket-communication.service';

@Component({
    'selector': 'app-p2p-handler',
    'templateUrl': './p2p-handler.component.html',
    'styleUrls': ['./p2p-handler.component.scss']
})
export class P2pHandlerComponent implements OnDestroy {
    private p2pSub: Subscription;

    isP2PActivated: boolean;

    constructor (private socketCommunication : SocketCommunicationService) {
        this.isP2PActivated = false;
        this.p2pSub = this.socketCommunication.subscribeP2PState.subscribe(this.changeP2PState.bind(this));
        console.log(this.isP2PActivated);
    }

    ngOnDestroy () {

        this.p2pSub.unsubscribe();

    }

    startP2P () {
        if (!this.isP2PActivated) {
            this.socketCommunication.startP2P();
        } else if (this.isP2PActivated) {
            console.log('P2P already activated');
        }
    }

    private changeP2PState (newState: boolean) {
        this.isP2PActivated = newState;
    }
}
