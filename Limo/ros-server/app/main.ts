import { Application } from './app';
import { Server } from './server';

const main = () => {
  const app = new Application();
  const server = new Server(app);
  server.init();
};

main();
