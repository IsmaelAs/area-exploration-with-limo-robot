import { Injectable } from '@angular/core';
import { io, Socket } from 'socket.io-client';
import { BACKEND_URL } from 'src/app/constants/url';
import RobotTargetType from 'src/app/types/RobotType';
import { DISTANCE_MOVEMENT, DIRECTION_MOVEMENT } from 'src/app/constants/robots-movement';
import RobotMovement from 'src/app/interfaces/robots-movement-interface';

@Injectable({
  providedIn: 'root'
})
export class SocketCommunicationService {

  private socket: Socket;

  constructor() {    
    this.socket = io(BACKEND_URL);
  }

  identify(robot: RobotTargetType){
    const movement: RobotMovement = {
      robot: robot,
      direction: DIRECTION_MOVEMENT.LEFT_FORWARD,
      distance: DISTANCE_MOVEMENT.FAR_AWAY
    }
    
    this.socket.emit("identify", movement);
  }

  startMission(robot: RobotTargetType) {

    const movement: RobotMovement = {
      robot: robot,
      direction: DIRECTION_MOVEMENT.FORWARD,
      distance: DISTANCE_MOVEMENT.CLOSE
    }
    console.log('start mission', movement);
    this.socket.emit("start-mission", movement)
  }
  
  stopMission(robot: RobotTargetType) {
    const movement: RobotMovement = {
      robot: robot,
      direction: DIRECTION_MOVEMENT.BACKWARD,
      distance: DISTANCE_MOVEMENT.CLOSE
    }

    this.socket.emit("stop-mission", movement)
  }
}
