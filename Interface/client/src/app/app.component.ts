import { Component } from '@angular/core';

@Component({
    'selector': 'app-root',
    'templateUrl': './app.component.html',
    'styleUrls': ['./app.component.scss']
})
export class AppComponent {

    isIpSet = false;

    setIsIpSet (event: { 'isIpSet': boolean }) {

        this.isIpSet = event.isIpSet;

    }

}
