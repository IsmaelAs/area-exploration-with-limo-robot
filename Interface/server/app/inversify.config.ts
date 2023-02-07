import { ServerController } from '@app/controllers/server.socket.controller';
import { ClientController } from '@app/controllers/client.socket.controller';
import {ConnectionService } from '@app/services/connection.service';
import { Container } from 'inversify';
import 'reflect-metadata';
import { Application } from './app';
import { Server } from './server';
import { TYPES } from './types';

export const containerBootstrapper: () => Promise<Container> = async () => {
    const container: Container = new Container();

    container.bind(TYPES.Server).to(Server);
    container.bind(TYPES.Application).to(Application);
    container.bind<ConnectionService>(TYPES.ConnectionService).to(ConnectionService);
    container.bind<ServerController>(TYPES.ServerController).to(ServerController);
    container.bind<ClientController>(TYPES.ClientController).to(ClientController);
    return container;
};
