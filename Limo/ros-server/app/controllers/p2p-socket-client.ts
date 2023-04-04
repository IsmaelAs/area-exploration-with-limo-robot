import { io, Socket } from 'socket.io-client';


export class P2PSocketClient {
  private socket: Socket;

  private p2pUrl: string;

  private p2pActivated: boolean;


  constructor(p2pUrl: string) {
    this.p2pActivated = false;
    this.p2pUrl = p2pUrl;
    this.socket = io(this.p2pUrl);

    this.initP2P();
  }

  activateP2P () {
    this.p2pActivated = true;
    this.emit('p2p-activated');
  }

  deactivateP2P () {
    this.p2pActivated = false;
    this.emit('p2p-deactivated');
  }

  initP2P() {
    this.socket.on('connect', () => {
      
      this.socket.on('reconnect', () => {
        window.location.reload();
      });
    });
  }

  private emit<T>(event: string, data?: T) {
    if (this.p2pActivated) data ? this.socket.emit(event, data) : this.socket.emit(event);
  }
}
