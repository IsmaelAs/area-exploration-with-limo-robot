import { Component } from '@angular/core';
import { Subscription } from 'rxjs';
import { SocketCommunicationService } from './services/socket-communication/socket-communication.service';

@Component({
    'selector': 'app-root',
    'templateUrl': './app.component.html',
    'styleUrls': ['./app.component.scss']
})
export class AppComponent {

    isIpSet = false;
    private stateSub: Subscription;
    state: string = "INIT";
    
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

    private changeState (newState: string) {
        this.state = newState;
    }

}
