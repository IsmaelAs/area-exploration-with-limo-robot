import { Component } from '@angular/core';
import RobotTargetType from 'src/app/types/RobotType';

@Component({
    'selector': 'app-robots-actions',
    'templateUrl': './robots-actions.component.html',
    'styleUrls': ['./robots-actions.component.scss']
})
export class RobotsActionsComponent {

    robotTarget: RobotTargetType = 'limo-1';

    listType: RobotTargetType[] = [
        'limo-1',
        'limo-2',
        'robots'
    ];

}
