import { injectable } from "inversify";
import { io, Socket } from "socket.io-client";
import { BACKEND_URL } from "../constants/url";
import { ServerSocketController } from "./server.socket.controller";
import Command from "@app/types/Command";

@injectable()
export class ClientSocketController {
    private client: Socket;
    private limoEmitter: ServerSocketController;

    constructor(serverSocket: ServerSocketController) {
        this.client = io(BACKEND_URL);
        this.limoEmitter = serverSocket
    }

    init() {
        this.client.on("connect", () => {
            console.log("Socket connected to backend");

            this.client.on("limo-move", (direction: Command, distance: number) => {
                this.limoEmitter.emit("move", direction, distance)
            });

            this.client.on("error", (err: Error) => {
            console.log(`client Client Error : ${err.stack}`);
            this.client.removeAllListeners()

            });

            this.client.on("disconnect", () => {
                console.log("Disconnected from limo robot");
                this.client.removeAllListeners()
            });
        });
    }
}
