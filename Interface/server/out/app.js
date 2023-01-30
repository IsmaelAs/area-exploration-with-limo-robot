"use strict";
var __decorate = (this && this.__decorate) || function (decorators, target, key, desc) {
    var c = arguments.length, r = c < 3 ? target : desc === null ? desc = Object.getOwnPropertyDescriptor(target, key) : desc, d;
    if (typeof Reflect === "object" && typeof Reflect.decorate === "function") r = Reflect.decorate(decorators, target, key, desc);
    else for (var i = decorators.length - 1; i >= 0; i--) if (d = decorators[i]) r = (c < 3 ? d(r) : c > 3 ? d(target, key, r) : d(target, key)) || r;
    return c > 3 && r && Object.defineProperty(target, key, r), r;
};
var __metadata = (this && this.__metadata) || function (k, v) {
    if (typeof Reflect === "object" && typeof Reflect.metadata === "function") return Reflect.metadata(k, v);
};
Object.defineProperty(exports, "__esModule", { value: true });
exports.Application = void 0;
const cookieParser = require("cookie-parser");
const cors = require("cors");
const express = require("express");
const http_status_codes_1 = require("http-status-codes");
const logger = require("morgan");
const swaggerJSDoc = require("swagger-jsdoc");
const swaggerUi = require("swagger-ui-express");
const typedi_1 = require("typedi");
const http_exception_1 = require("./classes/http.exception");
let Application = class Application {
    constructor() {
        this.internalError = http_status_codes_1.StatusCodes.INTERNAL_SERVER_ERROR;
        this.app = express();
        this.swaggerOptions = {
            swaggerDefinition: {
                openapi: '3.0.0',
                info: {
                    title: 'Cadriciel Serveur',
                    version: '1.0.0',
                },
            },
            apis: ['**/*.ts'],
        };
        this.config();
        this.bindRoutes();
    }
    bindRoutes() {
        this.app.use('/api/docs', swaggerUi.serve, swaggerUi.setup(swaggerJSDoc(this.swaggerOptions)));
        /*this.app.use('/classic', this.httpRoomsHandler.router);
        this.app.use('/best-scores', this.httpScoresHandler.router);
        this.app.use('/admin/gamesHistory', this.httpGamesHistoryHandler.router);
        this.app.use('/admin/virtualPlayers', this.httpVirtualPlayerDBHandler.router);
        this.app.use('/admin/dictionaries', this.httpDictionaryHandler.router);*/
        this.app.use('/', (req, res) => {
            res.redirect('/api/docs');
        });
        this.errorHandling();
    }
    config() {
        // Middlewares configuration
        this.app.use(logger('dev'));
        this.app.use(express.json());
        this.app.use(express.urlencoded({ extended: true }));
        this.app.use(cookieParser());
        this.app.use(cors());
    }
    errorHandling() {
        // When previous handlers have not served a request: path wasn't found
        this.app.use((req, res, next) => {
            const err = new http_exception_1.HttpException('Not Found');
            next(err);
        });
        // development error handler
        // will print stacktrace
        if (this.app.get('env') === 'development') {
            this.app.use((err, req, res) => {
                res.status(err.status || this.internalError);
                res.send({
                    message: err.message,
                    error: err,
                });
            });
        }
        // production error handler
        // no stacktraces leaked to user (in production env only)
        this.app.use((err, req, res) => {
            res.status(err.status || this.internalError);
            res.send({
                message: err.message,
                error: {},
            });
        });
    }
};
Application = __decorate([
    (0, typedi_1.Service)(),
    __metadata("design:paramtypes", [])
], Application);
exports.Application = Application;
//# sourceMappingURL=app.js.map