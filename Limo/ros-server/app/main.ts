import { Server } from "./server";
import { Application } from "./app";

const main = async () => {
    const app = new Application()
    const server = new Server(app);
    server.init()
}

main()
