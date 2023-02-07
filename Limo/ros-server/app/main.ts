import { SocketClient } from "./classes/socket-client";


const main = async () => {
    const socketClient = new SocketClient();
    await socketClient["nodeManager"]["nodeMouvement"].init();
    socketClient["nodeManager"].move("forward");

}

main()
