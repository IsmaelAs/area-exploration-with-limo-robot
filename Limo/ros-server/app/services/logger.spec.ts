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
        const spyNodeScan = sinon.spy(nodeScan, "initNodeScan")
        const mockOdom: Odometry = {
            pose: {
                pose: {
                    position: {
                        x: 1,
                        y: 2,
                        z: 3
                    }
                }
            }
        } as unknown as Odometry
        sinon.stub(logger['nodePosition'], 'getData').callsFake(() => {
            return mockOdom
        })

        logger.startLogs()
        clock.tick(1000)
        expect(spyNodePos.calledOnce)
        expect(spyNodeScan.calledOnce)
    });

    it("should return null when data is undefined", () => {
        const mockOdom: Odometry = {} as unknown as Odometry
        sinon.stub(nodePosition, 'getData').callsFake(() => {
            return mockOdom
        })
        logger.startLogs()
        clock.tick(1000)
    });

    it('should setLimoId', () => {
        logger.setLimoId(5)
        expect(logger["limoId"]).to.deep.equal(5);
    });

    it('should call asObservable when we call getObservable', () => {
        const spyOn = sinon.spy(logger["logsObservable"], "asObservable")
        logger.logObservable
        expect(spyOn.called).to.be.true;
    });

    it("should call closeNodePos and closeNodeScan when stopping logs", () => {
        const spyNodePos = sinon.spy(nodePosition, "closeNodePosition")
        const spyNodeScan = sinon.spy(nodeScan, "closeNodeScan")
        const spyOn = sinon.spy(global, 'clearInterval');
        const spyNext = sinon.spy(logger["logsObservable"], "next")
        logger.stopLog()

        expect(spyNodePos.calledOnce)
        expect(spyNodeScan.calledOnce)
        expect(spyOn.called).to.be.true;
        expect(spyNext.called).to.be.true;
    });


})