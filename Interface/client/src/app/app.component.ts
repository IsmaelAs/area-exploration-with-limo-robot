import { Component } from '@angular/core';
import { Subscription } from 'rxjs';
import { StateType } from './interfaces/state-limo';
import { SocketCommunicationService } from './services/socket-communication/socket-communication.service';

@Component({
    'selector': 'app-root',
    'templateUrl': './app.component.html',
    'styleUrls': ['./app.component.scss']
})
export class AppComponent {

    isIpSet = false;
    private stateSub: Subscription;
    stateLimo1: string = "INIT";
    stateLimo2: string = "INIT";
    
    constructor (
        private socketCommunication : SocketCommunicationService
      ) {
  
          this.stateSub = this.socketCommunication.subscribeState.subscribe(this.changeState.bind(this));
  
      }
    ngOnDestroy () {

        this.stateSub.unsubscribe();

    }
    setIsIpSet (event: { 'isIpSet': boolean }) {

        this.isIpSet = event.isIpSet;

    }

    private changeState (newState: StateType) {
        console.log("ICI JE CHANGE L'ETAT DANS L'AFFICHAGE");
        console.log(newState);
        newState.limoId === 1 ? this.stateLimo1 = newState.state : this.stateLimo2 = newState.state;
    }

}
