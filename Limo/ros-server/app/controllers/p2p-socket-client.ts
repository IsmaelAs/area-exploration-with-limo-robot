import { io, Socket } from 'socket.io-client';


export class P2PSocketClient {
  private socket: Socket;

  private p2pUrl: string;


  constructor(p2pUrl: string) {
    this.p2pUrl = p2pUrl;
    this.socket = io(p2pUrl);
  }


  initP2P() {
    this.socket.on('connect', () => {
      this.emit('p2p-connection');

      this.socket.on('reconnect', () => {
        window.location.reload();
      });
    });
  }

  private emit<T>(event: string, data?: T) {
    data ? this.socket.emit(event, data) : this.socket.emit(event);
  }
}
