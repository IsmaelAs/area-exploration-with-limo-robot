import { Component, OnInit } from '@angular/core';
import { firstValueFrom } from 'rxjs';
import MissionInfos from 'src/app/interfaces/mission-infos';
import { CommunicationService } from 'src/app/services/communication.service';

@Component({
    'selector': 'app-mission-action',
    'templateUrl': './mission-action.component.html',
    'styleUrls': ['./mission-action.component.scss']
})
export class MissionActionComponent implements OnInit {
    missions: MissionInfos[] | undefined = [];

    constructor (private commHTTP: CommunicationService) {}

    async ngOnInit (): Promise<void> {
        await this.setMissions();
    }

    async setMissions (): Promise<void> {
        try {
            this.missions = await firstValueFrom(this.commHTTP.getMissions());
        } catch (error) {
            // Pass
        }
    }

    sortMissions (property: keyof MissionInfos): void {
        this.missions?.sort((a, b) => {
            const aValue = a[property];
            const bValue = b[property];

            if (typeof aValue === 'number' && typeof bValue === 'number') {
                return aValue - bValue;
            }
            return String(aValue).localeCompare(String(bValue));

        });
    }
}
