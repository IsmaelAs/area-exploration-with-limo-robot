import { NodeMovement } from "./node-movement"
import * as sinon from "sinon";
import { beforeEach, describe, it } from "mocha";
import { RosMock } from "../../../classes/RosMock";
import { TopicMock } from "../../../classes/TopicMock";
import { BRIDGE_URI } from "../../../constants/url";
import * as roslibjs from 'roslib';
import { expect } from "chai";
import Command from '../../../types/types';
//import Twist from '../../../interfaces/Twist';
//import { Topic } from 'roslib';


describe("Node Movement Unittest", () => {
    let nodeMovement: NodeMovement
    let rosMock: RosMock
    let topicMock: TopicMock
    let clock: sinon.SinonFakeTimers

    beforeEach(() => {
        nodeMovement = new NodeMovement();
        rosMock = new RosMock({url: BRIDGE_URI})
        clock = sinon.useFakeTimers()
        topicMock = new TopicMock({
            ros: rosMock,
            name: 'cmd/vel',
            messageType: 'geometry_msgs/Twist',
            queue_size: 10,
        })

        sinon.stub(roslibjs, 'Ros').callsFake((args) => {
            return rosMock
        })
    
        sinon.stub(roslibjs, 'Topic').callsFake((args) => {
            return topicMock
        })    

    })

    afterEach(() => {
        sinon.restore()
        clock.restore()
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
        sinon.stub(nodeMovement, <any>'sendMsg').callsFake(async () => {
            return Promise.resolve();
        });
        const commandTest: Command = 'forward'
        const nbrTest = 5
        const spyOn = sinon.spy(nodeMovement, <any>'moveForward')
        await nodeMovement.move(commandTest, nbrTest)
        expect(spyOn.calledOnce)
    })

    it("should call moveBackward to make the limo move", async () => {
        sinon.stub(nodeMovement, <any>'sendMsg').callsFake(async () => {
            return Promise.resolve();
        });
        const commandTest: Command = 'backward'
        const nbrTest = 5
        const spyOn = sinon.spy(nodeMovement, <any>'moveBackward')
        await nodeMovement.move(commandTest, nbrTest)
        expect(spyOn.calledOnce)
    })

    it("should call turnLeftForward to make the limo move", async () => {
        sinon.stub(nodeMovement, <any>'sendMsg').callsFake(async () => {
            return Promise.resolve();
        });
        const commandTest: Command = 'left-forward'
        const nbrTest = 5
        const spyOn = sinon.spy(nodeMovement, <any>'turnLeftForward')
        await nodeMovement.move(commandTest, nbrTest)
        expect(spyOn.calledOnce)
    })

    it("should call turnRightForward to make the limo move", async () => {
        sinon.stub(nodeMovement, <any>'sendMsg').callsFake(async () => {
            return Promise.resolve();
        });
        const commandTest: Command = 'right-forward'
        const nbrTest = 5
        const spyOn = sinon.spy(nodeMovement, <any>'turnRightForward')
        await nodeMovement.move(commandTest, nbrTest)
        expect(spyOn.calledOnce)
    })

    it("should call turnLeftBackward to make the limo move", async () => {
        sinon.stub(nodeMovement, <any>'sendMsg').callsFake(async () => {
            return Promise.resolve();
        });
        const commandTest: Command = 'left-backward'
        const nbrTest = 5
        const spyOn = sinon.spy(nodeMovement, <any>'turnLeftBackward')
        await nodeMovement.move(commandTest, nbrTest)
        expect(spyOn.calledOnce)
    })

    it("should call turnRightBackward to make the limo move", () => {
        /*sinon.stub(nodeMovement, <any>'sendMsg').callsFake(async () => {
            return Promise.resolve();
        });*/
        nodeMovement['publisherMovement'] = new roslibjs.Topic({ros: new roslibjs.Ros({url: undefined}),
            name: 'cmd/vel',
            messageType: 'geometry_msgs/Twist',})
        sinon.stub(nodeMovement['publisherMovement'], 'publish').callsFake(() => {})
        const commandTest: Command = 'right-backward'
        const nbrTest = 5
        const spyOn = sinon.stub(nodeMovement, <any>'turnRightBackward')
        nodeMovement.move(commandTest, nbrTest)
        clock.tick(1000)
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
        const spyPublish = sinon.spy(topicMock, "publish")
        nodeMovement['publisherMovement'] = new Topic({ros: nodeMovement['ros'],
            name: nodeMovement['name'],
            messageType: 'std_msgs/String',
            queue_size: 10,})
        await nodeMovement['sendMsg'](nbrTest, dataTest)
        clock.tick(1000)
        expect(spyPublish.calledTwice)
    })*/
})