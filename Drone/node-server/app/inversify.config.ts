import { ServerSocketController } from '@app/controllers/server.socket.controller';
import { ClientSocketController } from '@app/controllers/client.socket.controller';
import { Container } from 'inversify';
import 'reflect-metadata';
import { Application } from './app';
import { Server } from './server';
import { TYPES } from './types';

export const containerBootstrapper: () => Promise<Container> = async () => {
    const container: Container = new Container();

    container.bind(TYPES.Server).to(Server);
    container.bind(TYPES.Application).to(Application);
    container.bind<ServerSocketController>(TYPES.ServerSocketController).to(ServerSocketController);
    container.bind<ClientSocketController>(TYPES.ClientSocketController).to(ClientSocketController);
    return container;
};
