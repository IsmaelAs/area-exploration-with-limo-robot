import { ROS_MASTER_URI } from '../../../constants/url'
import { BehaviorSubject } from 'rxjs'

const ros = require('rosnodejs')
// const roslib = require("roslib")

export class NodeMouvement {

    publisherMouvement: any
    subscriberMouvement: any
    master: any

    valuePassor: BehaviorSubject<unknown>

    // async init() {
    //     this.master = new roslib.Ros({ url: "ws://localhost:9090" })


    //     this.master.on('connection', function() {
    //             console.log('Connected to websocket server.');
    //         });
            
    //         this.master.on('error', function(error:Error) {
    //             console.log('Error connecting to websocket server: ', error);
    //         });
            
    //         this.master.on('close', function() {
    //             console.log('Connection to websocket server closed.');
    //         });

    //     this.publisherMouvement = new roslib.Topic({
    //         ros: this.master,
    //         name: "/cmd_vel",
    //         messageType: "geometry_msgs/Twist"
    //     })

    // }
    async init(){
        await ros.initNode('mouvement_node', {rosMasterUri: ROS_MASTER_URI}).then(() => {
        console.log('allo')
        const nh = ros.nh
        this.publisherMouvement = nh.advertise('cmd_vel', 'geometry_msgs/Twist')
        this.subscriberMouvement = nh.subscribe('rosout', 'rosgraph_msgs/Log', (msg: any) => {
            this.valuePassor.next(msg)
        })
    })}

    // move(msg: String) {
    //     console.log(msg);
    //     const msg2 = new roslib.Message({
    //         linear : {
    //             x : 1,
    //             y : 0,
    //             z : 0
    //         }
    //     });
    //     console.log(msg2)
    //     for (let i =0; i<10; i++)
    //         this.publisherMouvement.publish(msg2)
    // }

    move(command: string) {
        switch (command) {
            case "forward":
                this.moveForward();
                break;
            case "backward":
                this.moveBackward();
                break;
            case "left":
                this.turnLeft();
                break;
            case "right":
                this.turnRight();
                break;
            default:
                console.log("Invalid movement command");
                break;
        }
    }

    moveForward() {
        console.log('Moving robot forward');
        let str = ros.require('geometry_msgs')
        const msg = new str.msg.Twist({ data: {
            linear : {
                            x : 1,
                            y : 0,
                            z : 0
                        }
        }})
        msg.header.frame_id = 'base'
        for (let i =0; i<10; i++)
            this.publisherMouvement.publish(msg)
}

    moveBackward() {
        console.log('Moving robot backward');
        this.publisherMouvement.publish({data: "move_backward"})
    }

    turnLeft() {
        console.log('Turning robot left');
        this.publisherMouvement.publish({data: "turn_left"})
    }

    turnRight() {
        console.log('Turning robot right');
        this.publisherMouvement.publish({data: "turn_right"})
    }

    subscribe() {
        this.valuePassor.asObservable()
    }
}
