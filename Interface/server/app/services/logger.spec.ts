import { beforeEach, describe, it } from "mocha";
import { Logger } from "./logger";
import * as fs from "fs"
import * as sinon from "sinon" 
import LogLimo from "../interfaces/log-limo";
import { Socket } from "socket.io";
import * as chai from 'chai';
import { expect } from 'chai';
import * as sinonChai from 'sinon-chai';

chai.use(sinonChai);

describe("Logger Backend Unittest's", () => {
  let logger: Logger
  
  beforeEach(() => { 
    logger = new Logger()
  })

  afterEach(() => {
    sinon.restore()
  })

  it("should work", () => {
    expect(logger).to.exist
  })

  // it("should call appendFile when we call saveLimoData, there is a mission", () => {
  //   const spyAppendFile = sinon.stub(fs, "appendFile").callsFake(() => {})

  //   const logLimo: LogLimo = {
  //     limoId: 1,
  //     data: "data"
  //   }

  //   const currentMission = 1

  //   logger["currentMission"] = currentMission
  //   logger["isMissionStop"] = false

  //   logger.saveLimoData(logLimo)

  //   expect(spyAppendFile.called).to.be.true
  // })

  it("should call appendFile when we call saveLimoData, there is a mission", () => {
    const spyAppendFile = sinon.spy(fs, "appendFile");
  
    const logLimo: LogLimo = {
      limoId: 1,
      data: "data"
    };
  
    const currentMission = 1;
  
    logger["currentMission"] = currentMission;
    logger["isMissionStop"] = false;
  
    logger.saveLimoData(logLimo);
  
    const logFilePath = `./app/logs/logs-${currentMission}.log`;
    const expectedLogContent = `[${new Date().toLocaleString()} : LIMO-${logLimo.limoId}] : ${JSON.stringify(logLimo.data)}\n`;
  
    sinon.assert.calledOnce(spyAppendFile);
    sinon.assert.calledWith(spyAppendFile, logFilePath, expectedLogContent);
  });
  
  it("should not call appendFile when we call saveLimoData there is no mission", () => {
    const spyAppendFile = sinon.stub(fs, "appendFile").callsFake(() => {})

    const logLimo: LogLimo = {
      limoId: 1,
      data: "data"
    }

    logger["currentMission"] = 0
    logger["isMissionStop"] = false


    logger.saveLimoData(logLimo)

    expect(spyAppendFile.called).to.be.false
  })

  it("should not call appendFile when we call saveLimoData there is no mission 2", () => {
    const spyAppendFile = sinon.stub(fs, "appendFile").callsFake(() => {})

    const logLimo: LogLimo = {
      limoId: 1,
      data: "data"
    }

    logger["currentMission"] = 1
    logger["isMissionStop"] = true


    logger.saveLimoData(logLimo)

    expect(spyAppendFile.called).to.be.false
  })

  // it("should call appendFile when we call saveUserData, there is a mission", () => {
  //   const spyAppendFile = sinon.stub(fs, "appendFile").callsFake(() => {})

  //   const logLimo = {
  //     data: "data"
  //   }

  //   const currentMission = 1

  //   logger["currentMission"] = currentMission
  //   logger["isMissionStop"] = false

  //   logger.saveUserData(logLimo)

  //   expect(spyAppendFile.called).to.be.true
  // })

  it("should call appendFile when we call saveUserData, there is a mission", () => {
    const spyAppendFile = sinon.spy(fs, "appendFile");
  
    const logLimo = {
      data: "data"
    };
  
    const currentMission = 1;
  
    logger["currentMission"] = currentMission;
    logger["isMissionStop"] = false;
  
    logger.saveUserData(logLimo);
  
    const logFilePath = `./app/logs/logs-${currentMission}.log`;
    const expectedLogContent = `[${new Date().toLocaleString()} : User] : ${JSON.stringify(logLimo)}\n`;
  
    sinon.assert.calledOnce(spyAppendFile);
    sinon.assert.calledWith(spyAppendFile, logFilePath, expectedLogContent);
  });
  

  it("should not call appendFile when we call saveUserData there is no mission", () => {
    const spyAppendFile = sinon.stub(fs, "appendFile").callsFake(() => {})

    const logLimo = {
      data: "data"
    }

    logger["currentMission"] = 0
    logger["isMissionStop"] = false


    logger.saveUserData(logLimo)

    expect(spyAppendFile.called).to.be.false
  })

  it("should not call appendFile when we call saveUserData there is no mission 2", () => {
    const spyAppendFile = sinon.stub(fs, "appendFile").callsFake(() => {})

    const logLimo = {
      limoId: 1,
      data: "data"
    }

    logger["currentMission"] = 1
    logger["isMissionStop"] = true


    logger.saveUserData(logLimo)

    expect(spyAppendFile.called).to.be.false
  })

  it("should not call readFile when there is not mission", () => {
    const spyReadFile = sinon.stub(fs, "readFile").callsFake(() => {})
    const fakeSocket: Socket = {emit: () => {}} as unknown as Socket
    const spySocket = sinon.stub(fakeSocket, "emit")

    logger["currentMission"] = 0

    const returnOfFunction = logger.getAllData(1, fakeSocket)

    expect(spyReadFile.called).to.be.false
    expect(spySocket.called).to.be.true
    expect(returnOfFunction).to.be.undefined
  })

  // it("should call readFile there is a mission when we call getAllData", () => {
  //   const spyReadFile = sinon.stub(fs, "readFile").callsFake(() => {})
  //   const fakeSocket: Socket = {emit: () => {}} as unknown as Socket

  //   logger["currentMission"] = 1

  //   logger.getAllData(1, fakeSocket)
    
  //   expect(spyReadFile.called).to.be.true
  // })


  it("should stop mission when there is a mission and we call stopMission", () => {
    logger["isMissionStop"] = false

    logger.stopMission()

    expect(logger["isMissionStop"]).to.be.true
  })

  it("should not stop mission when there is not a mission and we call stopMission", () => {
    logger["isMissionStop"] = true

    logger.stopMission()

    expect(logger["isMissionStop"]).to.be.true

  })

  it("should increment currentMission and set isMissionStop when there is not a mission and we call startMission", () => {
    logger["currentMission"] = 4
    logger["isMissionStop"] = true

    logger.startMission()

    expect(logger["currentMission"]).to.be.equal(5)
    expect(logger["isMissionStop"]).to.be.false
  })

  it("should not increment currentMission and set isMissionStop when there is a mission and we call startMission", () => {
    logger["currentMission"] = 4
    logger["isMissionStop"] = false

    logger.startMission()

    expect(logger["currentMission"]).to.be.equal(4)
    expect(logger["isMissionStop"]).to.be.false

  })
  it("should call appendFile when we call saveLimoData, there is a mission", () => {
    const spyAppendFile = sinon.stub(fs, "appendFile").callsFake(() => {});

    const logLimo: LogLimo = {
      limoId: 1,
      data: "data",
    };

    logger["currentMission"] = 1;
    logger["isMissionStop"] = false;

    logger.saveLimoData(logLimo);

    expect(spyAppendFile.called).to.be.true;
  });

  it("should call appendFile when we call saveUserData, there is a mission", () => {
    const spyAppendFile = sinon.stub(fs, "appendFile").callsFake(() => {});

    const logLimo = {
      data: "data",
    };

    logger["currentMission"] = 1;
    logger["isMissionStop"] = false;

    logger.saveUserData(logLimo);

    expect(spyAppendFile.called).to.be.true;
  });

  it("should call readFile when there is a mission and we call getAllData", () => {
    const spyReadFile = sinon.stub(fs, "readFile").callsFake(() => {});
    const fakeSocket: Socket = { emit: () => {} } as unknown as Socket;

    logger["currentMission"] = 1;

    logger.getAllData(1, fakeSocket);

    expect(spyReadFile.called).to.be.true;
  });

  it("should emit error when readFile returns an error", () => {
    sinon.stub(fs, "readFile").callsFake((path, callback) => {
      callback(new Error("Test error"), <any>null);
    });
    const fakeSocket: Socket = { emit: () => {} } as unknown as Socket;
    const spySocket = sinon.stub(fakeSocket, "emit");

    logger["currentMission"] = 1;

    logger.getAllData(1, fakeSocket);

    expect(spySocket.calledWith("send-all-logs", sinon.match.string)).to.be.true;
  });

  it("should emit data when readFile returns data", () => {
    const testData = "Test data";
    sinon.stub(fs, "readFile").callsFake((path, callback) => {
      callback(null, Buffer.from(testData, "ascii"));
    });
    const fakeSocket: Socket = { emit: () => {} } as unknown as Socket;
    const spySocket = sinon.stub(fakeSocket, "emit");

    logger["currentMission"] = 1;

    logger.getAllData(1, fakeSocket);

    expect(spySocket.calledWith("send-all-logs", testData)).to.be.true;
  });

})