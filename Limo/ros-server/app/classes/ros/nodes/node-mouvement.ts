// import { ROS_MASTER_URI } from '../../../constants/url'
import { BehaviorSubject } from 'rxjs'

// const ros = require('rosnodejs')
const roslib = require("roslib")

export class NodeMouvement {

    publisherMouvement: any
    subscriberMouvement: any
    master: any

    valuePassor: BehaviorSubject<unknown>

    async init() {
        this.master = new roslib.Ros({ url: "http://localhost:11311" })
        this.publisherMouvement = new roslib.Topic({
            ros: this.master,
            name: "/rosout",
            messageType: "rosgraph_msgs/Log"
        })
    }
    // async init(){
    //     await ros.initNode('mouvement_node', {rosMasterUri: ROS_MASTER_URI}).then(() => {
    //     console.log('allo')
    //     const nh = ros.nh
    //     this.publisherMouvement = nh.advertise('rosout', 'rosgraph_msgs/Log')
    //     this.subscriberMouvement = nh.subscribe('rosout', 'rosgraph_msgs/Log', (msg: any) => {
    //         this.valuePassor.next(msg)
    //     })
    // })}

    move(msg: String) {
        console.log(msg);
        const msg2 = new roslib.Message({
            header:
               {
                seq: 1,
                stamp:
                    {
                        secs: 12345678,
                        nsecs: 98765432
                    },
                frame_id: "/frame_id_example"
               },
            level: 1,
            name: "example_node",
            msg: "This is a log message example",
            file: "example_file.cpp",
            function: "example_function",
            line: 123,
            topics: ["/topic_1", "/topic_2"]

        })
        this.publisherMouvement.publish(msg2)
    }

    // move(command: string) {
    //     switch (command) {
    //         case "forward":
    //             this.moveForward();
    //             break;
    //         case "backward":
    //             this.moveBackward();
    //             break;
    //         case "left":
    //             this.turnLeft();
    //             break;
    //         case "right":
    //             this.turnRight();
    //             break;
    //         default:
    //             console.log("Invalid movement command");
    //             break;
    //     }
    // }

    // moveForward() {
    //     console.log('Moving robot forward');
    //     let str = ros.require('rosgraph_msgs')
    //     const msg = new str.msg.Log()
    //     this.publisherMouvement.publish(msg)
    // }

    // moveBackward() {
    //     console.log('Moving robot backward');
    //     this.publisherMouvement.publish({data: "move_backward"})
    // }

    // turnLeft() {
    //     console.log('Turning robot left');
    //     this.publisherMouvement.publish({data: "turn_left"})
    // }

    // turnRight() {
    //     console.log('Turning robot right');
    //     this.publisherMouvement.publish({data: "turn_right"})
    // }

    // subscribe() {
    //     this.valuePassor.asObservable()
    // }
}
