import { Server as SocketServer } from 'socket.io';
import { injectable } from 'inversify';

@injectable()
export class ServerSocketController {
    private io: SocketServer;

    constructor(io: SocketServer) {
        this.io = io;
    }

    init() {
      this.io.on("connection", (socket) => {
        console.log("Socket Server Connected !");

        socket.on("error", (err: Error) => {
        console.log(`Io : Io Error : ${err.stack}`);
        });

        socket.on("disconnect", () => {
          console.log("Disconnected from limo robot");
        });
      });
    }

    emit<T>(event: string, data?: T) {
        data ? this.io.emit(event, data) : this.io.emit(event)
    }
}