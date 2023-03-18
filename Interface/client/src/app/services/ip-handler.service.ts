import { Injectable } from '@angular/core';
import { SocketCommunicationService } from './socket-communication/socket-communication.service';

@Injectable({
    'providedIn': 'root'
})
export class IpHandlerService {

    private ipLimo1: string = '';

    private ipLimo2 : string = '';


    constructor (private socketCommunication: SocketCommunicationService) { }

    setIps (ipLimo1: string, ipLimo2: string) {

        this.socketCommunication.sendLimoIps(ipLimo1, ipLimo2);
        this.ipLimo1 = ipLimo1;
        this.ipLimo2 = ipLimo2;

    }

    get ipAddressLimo1 () {

        return this.ipLimo1;

    }

    get ipAddressLimo2 () {

        return this.ipLimo2;

    }

}
