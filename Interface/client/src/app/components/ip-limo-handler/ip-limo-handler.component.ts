import { Component, Output, EventEmitter } from '@angular/core';
import { IpHandlerService } from 'src/app/services/ip-handler.service';

@Component({
    'selector': 'app-ip-limo-handler',
    'templateUrl': './ip-limo-handler.component.html',
    'styleUrls': ['./ip-limo-handler.component.scss']
})
export class IpLimoHandlerComponent {

    ipAddressLimo1 = '';

    ipAddressLimo2 = '';

    @Output() isIpSet = new EventEmitter<{ 'isIpSet': boolean }>();

    constructor (private ipHandler: IpHandlerService) {}

    sendLimoIps () {

        this.ipHandler.setIps(this.ipAddressLimo1, this.ipAddressLimo2);
        this.isIpSet.emit({ 'isIpSet': true });

    }

    get ipPatternRight () {

        const MAX_IP = 256;
        const MIN_IP = -1;
        const IP_LENGTH = 4;

        const ips = this.ipAddressLimo1.split('.');

        return this.ipAddressLimo1 !== '' && ips.length === IP_LENGTH && ips.every((ip) => ip !== '' && Number(ip) < MAX_IP && Number(ip) > MIN_IP);

    }

}
