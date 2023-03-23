import { State } from '../types/States';
import { setInterval } from 'timers';
import StateType from '@app/types/StateType';
import { Subject } from 'rxjs';


export class MyStateMachine {

  private currentState: State = "INIT";
  private limoId: number;
  private statesObservable: Subject<StateType> = new Subject();
  intervalState: any;
 


  private callBack(data: {percentage: number}) {
    if (!process.env.IS_SIMULATION) {
      // handle low battery in physical mode
      //this.onLowBattery(data?.percentage);
    }
      this.statesObservable.next({limoId: this.limoId, state: this.currentState});
  }
 


  startStates() {
    this.intervalState = setInterval(this.callBack.bind(this), 1000)
  }

//  onLowBattery(percentage: number = 100): void {
//     // Check if the battery is below 30%
//     if (percentage < 30) {
//       console.log('Battery level is below 30%');
//       this.currentState = "STOPPED";
//     } else if (this.currentState !== "ON_MISSION"  && this.currentState !== "WAITING") {
//         this.currentState = "WAITING";
//     }
//  }

 setLimoId(limoId: number) {
  this.limoId = limoId;
}

  onMission() {
    if (this.currentState === "WAITING")
      this.currentState = "ON_MISSION";
  }
  
  onReady() {
    this.currentState = "WAITING";
  }
  onMissionEnd() {
    console.log("ICI JE MET FIN A LA MISSION-STOPPED")
    this.currentState = "STOPPED";
  }

  stopStates() {
    clearInterval(this.intervalState);
  }
 
  get stateObservable() {
    return this.statesObservable.asObservable();
  }
}


