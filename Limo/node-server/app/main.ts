//Root of the code, everything starts from here*
// WARNING : Make sure to always import 'reflect-metadata' and 'module-alias/register' first
//import console = require('console');
import 'module-alias/register';
import 'reflect-metadata';

import { Server } from './server';
import { Container } from 'typedi';

const server: Server = Container.get(Server);
server.init();



