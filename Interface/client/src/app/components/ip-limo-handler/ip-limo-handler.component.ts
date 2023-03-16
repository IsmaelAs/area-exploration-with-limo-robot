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


        let isIp2Good = true;
        if (this.ipAddressLimo2.replace(' ', '') !== '') {

            const ips2 = this.ipAddressLimo2.split('.');
            isIp2Good = ips2.length === IP_LENGTH && ips2.every((ip) => ip !== '' && Number(ip) < MAX_IP && Number(ip) > MIN_IP);

        }

        const ips1 = this.ipAddressLimo1.split('.');
        return isIp2Good && this.ipAddressLimo1 !== '' && ips1.length === IP_LENGTH && ips1.every((ip) => ip !== '' && Number(ip) < MAX_IP && Number(ip) > MIN_IP);

    }

}
