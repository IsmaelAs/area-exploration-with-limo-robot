import { BRIDGE_URI } from '../constants/url';
import { Ros, Topic } from 'roslib';
import { State } from '../types/States';
//import { SocketServer } from '../controllers/socket-server';
import { setInterval } from 'timers';
import StateType from '@app/types/StateType';
import { Subject } from 'rxjs';


export class MyStateMachine {

  private currentState: State = "INIT";
  private ros: Ros;
  //private socketServer: SocketServer;
  private limoId: number;
  private statesObservable: Subject<StateType> = new Subject();


  BatteryPercentage: any;
  onBatteryMessage: any;
  intervalState: any;
 


  constructor() {
    // Connexion au serveur ROS
    this.ros = new Ros({
      url: BRIDGE_URI,
    });
 
    this.callBack = this.callBack.bind(this);

      // Abonnement au topic "battery_state"
    const batterySubscriber = new Topic({
        ros: this.ros,
        name: '/battery_state',
        messageType: 'sensor_msgs/BatteryState',
      });
    batterySubscriber.subscribe(this.callBack.bind(this));
    }
 


  private callBack(data: {percentage: number}) {
    if (!process.env.IS_SIMULATION) {
      // handle low battery in physical mode
      this.onLowBattery(data?.percentage);
    }
      //const limoId = this.limoId;
      this.statesObservable.next({limoId: this.limoId, state: this.currentState});
      //this.socketServer.emit("save-state", {limoId: limoId, state: this.currentState})
  }
 


  startStates() {
    this.intervalState = setInterval(this.callBack, 1000)
  }
 


 onLowBattery(percentage: number = 100): void {
    // Check if the battery is below 30%
    if (percentage < 30) {
      console.log('Battery level is below 30%');
      this.currentState = "STOPPED";
    } else if (this.currentState !== "ON_MISSION"  && this.currentState !== "WAITING") {
        this.currentState = "WAITING";
    }
 }

 setLimoId(limoId: number) {
  this.limoId = limoId;
}

  onMission() {
    this.currentState = "ON_MISSION";
  }
  onMissionEnd() {
    this.currentState = "STOPPED";
  }

  stopStates() {
    clearInterval(this.intervalState);
  }
 
  get stateObservable() {
    return this.statesObservable.asObservable();
  }
}


