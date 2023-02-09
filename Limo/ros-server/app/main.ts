import { SocketClient } from "./classes/socket-client";


const main = async () => {
    const socketClient = new SocketClient();
    socketClient.connect()
}

main()
