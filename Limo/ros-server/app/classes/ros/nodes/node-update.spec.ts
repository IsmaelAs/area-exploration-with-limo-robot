import { describe, beforeEach } from "mocha";
import * as sinon from 'sinon'
import * as roslibjs from 'roslib'
import { expect } from "chai";
import { RosMock } from "../../../classes/RosMock";
import { TopicMock } from "../../../classes/TopicMock";
import { BRIDGE_URI } from "../../../constants/url";
import { NodeUpdate } from "./node-update";



describe("Node Update Unittest's", ()=> {
    let nodeUpdate: NodeUpdate
    let rosMock: RosMock
    let topicMock: TopicMock


    beforeEach(() => {
        rosMock = new RosMock({url: BRIDGE_URI})

        topicMock = new TopicMock({
            ros: rosMock,
            name: 'restartPackagesContainer',
            messageType: 'std_msgs/Bool',
            queue_size: 10,
        })

        sinon.stub(roslibjs, 'Ros').callsFake((args) => {
            return rosMock
        })

        sinon.stub(roslibjs, 'Topic').callsFake((args) => {
            return topicMock
        })  

        nodeUpdate = new NodeUpdate()

    })

    afterEach(() => {
        sinon.restore()
    })

    it("should work", () => {
        expect(nodeUpdate).to.exist
    })

    it("should initialize node when we call initNodeScan", () => {
        const spyOn = sinon.spy(rosMock, "on")
        nodeUpdate.initNodeScan()
        expect(spyOn.called).to.be.true
        expect(nodeUpdate["ros"]).to.deep.equal(rosMock)
        expect(nodeUpdate["publisherUpdate"]).to.deep.equal(topicMock)
    })


   
    it("should restartContainers ", () => {
        nodeUpdate.initNodeScan()
        let stubProcess = sinon.stub(process, "exit").callsFake((code?:number)=>{ return 0 as unknown as never})
        const consoleSpy = sinon.spy(console, 'log')
        nodeUpdate.restartContainers()
        expect(stubProcess.called).to.be.true
        expect(consoleSpy.called).to.be.true
    })

    it("should call close when we call closeNodeUpdate", () => {
        const spyClose = sinon.spy(rosMock, "close")
        // to init ros in the node
        nodeUpdate["ros"] = rosMock as unknown as roslibjs.Ros
        nodeUpdate.closeNodeUpdate()
        expect(spyClose.called).to.be.true
    })
})