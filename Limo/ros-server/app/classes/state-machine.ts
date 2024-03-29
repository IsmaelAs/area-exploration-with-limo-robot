import { State } from '../types/States';
import { setInterval } from 'timers';
import StateType from '@app/types/StateType';
import { Subject } from 'rxjs';


export class MyStateMachine {
  private currentState: State = 'INIT';

  private limoId: number;

  private statesObservable: Subject<StateType> = new Subject();

  intervalState: NodeJS.Timer;


  private callBack() {
    if (!process.env.IS_SIMULATION) {

      /*
       *  Handle low battery in physical mode
       * this.onLowBattery(data?.percentage);
       */
    }
    this.statesObservable.next({
      limoId: this.limoId,
      state: this.currentState,
    });
  }


  startStates() {
    this.intervalState = setInterval(this.callBack.bind(this), 1000);
  }


  setLimoId(limoId: number) {
    this.limoId = limoId;
  }

  onMission() {
    if (this.currentState === 'WAITING') {
      this.currentState = 'ON_MISSION';
    }
  }

  onReady() {
    this.currentState = 'WAITING';
  }

  onMissionEnd() {
    console.log('ICI JE MET FIN A LA MISSION-STOPPED');
    this.currentState = 'STOPPED';
  }

  stopStates() {
    clearInterval(this.intervalState);
  }

  get stateObservable() {
    return this.statesObservable.asObservable();
  }
}


