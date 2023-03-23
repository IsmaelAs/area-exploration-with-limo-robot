//import StateLimo from '@app/interfaces/state-limo';
import { Socket } from 'socket.io';


export class State {

  getAllData(data: string, socket: Socket): void {
        socket.emit('send-state', data);
      }
}
