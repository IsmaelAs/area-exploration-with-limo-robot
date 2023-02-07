"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
//Root of the code, everything starts from here*
// WARNING : Make sure to always import 'reflect-metadata' and 'module-alias/register' first
//import console = require('console');
require("module-alias/register");
require("reflect-metadata");
const server_1 = require("./server");
const typedi_1 = require("typedi");
const server = typedi_1.Container.get(server_1.Server);
server.init();
//# sourceMappingURL=main.js.map