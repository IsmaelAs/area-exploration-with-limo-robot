import { Component, Input } from '@angular/core';
import { SocketCommunicationService } from 'src/app/services/socket-communication/socket-communication.service';

@Component({
    'selector': 'app-update-component',
    'templateUrl': './update-component.component.html',
    'styleUrls': ['./update-component.component.scss']
})
export class UpdateComponentComponent {
  @Input() stateLimo1 = 'INIT';

  @Input() stateLimo2 = 'INIT';

  constructor (private commSocket: SocketCommunicationService) {}

  updateLimo1 () {
      console.log('update limo 1');

      this.commSocket.updateLimo('limo-1');
  }

  updateLimo2 () {
      console.log('update limo 2');

      this.commSocket.updateLimo('limo-2');
  }
}
