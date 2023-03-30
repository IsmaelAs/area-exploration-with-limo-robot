import { SocketServer } from '@app/controllers/socket-server';


export class P2PManager {
  limoId: number;

  socketServer: SocketServer;

  P2PSocket: 

  constructor(socketServer: SocketServer) {
    this.socketServer = socketServer;
    this.limoId = socketServer.limoId;
    this.initP2P();
  }

  initP2P() {

  }
}
