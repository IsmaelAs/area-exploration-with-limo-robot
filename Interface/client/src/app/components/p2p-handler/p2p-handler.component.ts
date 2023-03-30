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

    private isP2PActivated: boolean;

    constructor (private socketCommunication : SocketCommunicationService) {
        this.isP2PActivated = false;
        this.p2pSub = this.socketCommunication.subscribeP2PState.subscribe(this.changeP2PState.bind(this));
    }

    ngOnDestroy () {

        this.p2pSub.unsubscribe();

    }

    private startP2P () {
        if (!this.isP2PActivated) {
            this.socketCommunication.startP2P();
        }
    }

    private changeP2PState (newState: boolean) {
        console.log(newState);
        this.isP2PActivated = newState;
    }
}
