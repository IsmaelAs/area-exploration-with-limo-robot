import { injectable } from "inversify";
import { io, Socket } from "socket.io-client";
import { BACKEND_URL } from "../constants/url";
import { ServerSocketController } from "./server.socket.controller";
import OnRobotMovement from "@app/interfaces/on-robots-movement-interface";

@injectable()
export class ClientSocketController {
    private client: Socket;
    private limoEmitter: ServerSocketController;

    constructor(serverSocket: ServerSocketController) {
        console.log('backend uri', BACKEND_URL)
        this.client = io(BACKEND_URL);
        console.log('sal super gens');
        
        this.limoEmitter = serverSocket
    }

    init() {
        this.client.on("connect", () => {
            console.log("Socket connected to backend");
            console.log(`limo-${process.env.LIMO_ID}-move`);
            
            this.client.on(`limo-${process.env.LIMO_ID}-move`, (movement: OnRobotMovement) => {
                console.log('move event received');
                this.limoEmitter.emit("move", movement)
                
            });

            this.client.on(`robots-move`, (movement: OnRobotMovement) => {
                console.log('move event received');
                this.limoEmitter.emit("move", movement)
                
            });

            this.client.on("error", (err: Error) => {
                console.log(`client Client Error : ${err.stack}`);
                this.client.removeAllListeners()

            });

            this.client.on("disconnect", () => {
                console.log("Client node socket disconnected");
                this.client.removeAllListeners()
            });
        });
    }
}
