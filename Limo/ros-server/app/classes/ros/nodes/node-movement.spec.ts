import { NodeMovement } from "./node-movement"
import * as sinon from "sinon";
import { beforeEach, describe, it } from "mocha";
import { RosMock } from "../../../classes/RosMock";
import { TopicMock } from "../../../classes/TopicMock";
import { BRIDGE_URI } from "../../../constants/url";
import * as roslibjs from 'roslib';
import { expect } from "chai";
import Command from '../../../types/types';

describe("Node Movement Unittest", () => {
    let nodeMovement: NodeMovement
    let rosMock: RosMock
    let topicMock: TopicMock

    beforeEach(() => {
        nodeMovement = new NodeMovement();
        rosMock = new RosMock({url: BRIDGE_URI})

        topicMock = new TopicMock({
            ros: rosMock,
            name: process.env.IS_SIMULATION ? `limo${process.env.LIMO_ID}/cmd_vel` : 'cmd_vel',
            messageType: 'geometry_msgs/Twist',
            queue_size: 10,
        })

        sinon.stub(roslibjs, 'Ros').callsFake((args) => {
            return rosMock
        })
    
        sinon.stub(roslibjs, 'Topic').callsFake((args) => {
            return topicMock
        })    

        sinon.stub(nodeMovement, <any>'sendMsg').callsFake(async () => {
            return Promise.resolve();
        });
    })

    afterEach(() => {
        sinon.restore()
    })

    it("should work", () => {
        expect(nodeMovement).to.exist
    })

    it("should initialize node when we call initNodeMovement", () => {
        const spyOn = sinon.spy(rosMock, "on")
        nodeMovement.initNodeMovement()
        expect(spyOn.called).to.be.true
    })

    it("Close connection to all nodes", () => {
        const spyOn = sinon.spy(rosMock, "close")
        nodeMovement.initNodeMovement()
        nodeMovement.closeNodeMovement()
        expect(spyOn.called).to.be.true
    })

    it("should call moveForward to make the limo move", async () => {
        const commandTest: Command = 'forward'
        const nbrTest = 5
        const spyOn = sinon.spy(nodeMovement, <any>'moveForward')
        await nodeMovement.move(commandTest, nbrTest)
        expect(spyOn.calledOnce)
    })

    it("should call moveBackward to make the limo move", async () => {
        const commandTest: Command = 'backward'
        const nbrTest = 5
        const spyOn = sinon.spy(nodeMovement, <any>'moveBackward')
        await nodeMovement.move(commandTest, nbrTest)
        expect(spyOn.calledOnce)
    })

    it("should call turnLeftForward to make the limo move", async () => {
        const commandTest: Command = 'left-forward'
        const nbrTest = 5
        const spyOn = sinon.spy(nodeMovement, <any>'turnLeftForward')
        await nodeMovement.move(commandTest, nbrTest)
        expect(spyOn.calledOnce)
    })

    it("should call turnRightForward to make the limo move", async () => {
        const commandTest: Command = 'right-forward'
        const nbrTest = 5
        const spyOn = sinon.spy(nodeMovement, <any>'turnRightForward')
        await nodeMovement.move(commandTest, nbrTest)
        expect(spyOn.calledOnce)
    })

    it("should call turnLeftBackward to make the limo move", async () => {
        const commandTest: Command = 'left-backward'
        const nbrTest = 5
        const spyOn = sinon.spy(nodeMovement, <any>'turnLeftBackward')
        await nodeMovement.move(commandTest, nbrTest)
        expect(spyOn.calledOnce)
    })

    it("should call turnRightBackward to make the limo move", async () => {
        const commandTest: Command = 'right-backward'
        const nbrTest = 5
        const spyOn = sinon.spy(nodeMovement, <any>'turnRightBackward')
        await nodeMovement.move(commandTest, nbrTest)
        expect(spyOn.calledOnce)
    })

    it("should do nothing when the command is invalid", async () => {
        const commandTest: Command = 'invalid'
        const nbrTest = 5
        await nodeMovement.move(commandTest, nbrTest)
    })

    /*it("should publish to the topic when sendMsg() is called", async () => {
        const dataTest: Twist = {
            linear: {
              x: 1,
            },
          }
        const nbrTest = 5
        

    })*/
})