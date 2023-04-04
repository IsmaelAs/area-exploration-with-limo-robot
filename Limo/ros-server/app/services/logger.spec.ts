import { Logger } from "./logger"
import * as sinon from "sinon";
import { beforeEach, describe, it } from "mocha";
import { NodePosition } from "./../classes/ros/nodes/node-position";
import NodeScan from "./../classes/ros/nodes/node-scan";
import Odometry from "./../types/Odometry/Odometry";
import { expect } from "chai";


describe("Logger Unittest", () => {
    
    let logger: Logger;
    let nodePosition: NodePosition
    let nodeScan: NodeScan
    let clock: sinon.SinonFakeTimers

    beforeEach(() => {
        logger = new Logger()
        nodePosition = new NodePosition()
        nodeScan = new NodeScan()
        clock = sinon.useFakeTimers();

    })

    afterEach(() => {
        sinon.restore()
        clock.restore()
    })

    it("should call initNodePos and initNodeScan when starting logs", () => {
        const spyNodePos = sinon.spy(nodePosition, "initNodePosition")
        const spyNodeScan = sinon.spy(nodeScan,"initNodeScan") 
        const returnOdom: Odometry = {} as unknown as Odometry
        sinon.stub(nodePosition, "getData").callsFake(() => {
            return returnOdom
        })
        logger.startLogs()
        clock.tick(1000)
        expect(spyNodePos.calledOnce)
        expect(spyNodeScan.calledOnce)
    })
})