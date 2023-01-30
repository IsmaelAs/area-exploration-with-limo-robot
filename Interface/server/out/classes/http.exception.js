"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.HttpException = void 0;
const http_status_codes_1 = require("http-status-codes");
class HttpException extends Error {
    constructor(message, status = http_status_codes_1.StatusCodes.INTERNAL_SERVER_ERROR) {
        super(message);
        this.status = status;
        this.name = 'HttpException';
    }
}
exports.HttpException = HttpException;
//# sourceMappingURL=http.exception.js.map