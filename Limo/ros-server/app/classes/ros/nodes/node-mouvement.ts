import delay from 'delay'
import {Topic, Ros, Message} from 'roslib'
import Twist from '../../../interfaces/Twist'
import { ROS_MASTER_URI } from '../../../constants/url'
import Command from "../../../types/Command"

export class NodeMouvement {
    private name: String = "Node Mouvement"

    private publisherMouvement: Topic
    private ros: Ros

    // Connect the node to the Limo
    initNodeMouvement(): void {
        this.ros = new Ros({ url: ROS_MASTER_URI })

        // Console error when error
        this.ros.on('error', (err: Error) => {
            console.error(`${this.name} : ${err.stack}`)
        })

        // Wait for ROS to connect to the bridge
        this.ros.on('connection', () => {

            // Initialise the publisher to the cmd_vel topic while ROS is connected
            this.publisherMouvement = new Topic({
                ros: this.ros,
                name: "/cmd_vel",
                messageType: "geometry_msgs/Twist",                
                queue_size: 10
            })

        });

        // Console when connection closed
        this.ros.on('close', () => {
            console.log(`${this.name} : Connection closed !`);
            
        })
    }

    // Close connection to all nodes
    closeNodeMouvement() {
        this.ros.close()
    }

    // Send command to make the limo move
    move(command: Command, nbrSendingMsg = 5): void {
        console.log(`${this.name} : Moving ${command}`);

        switch (command) {
            case "forward":
                this.moveForward(nbrSendingMsg);
                break;
            case "backward":
                this.moveBackward(nbrSendingMsg);
                break;
            case "left-forward":
                this.turnLeftForward(nbrSendingMsg);
                break;
            case "right-forward":
                this.turnRightForward(nbrSendingMsg);
                break;
            case "right-backward":
                this.turnRightBackward(nbrSendingMsg);
                break;
            case "left-forward":
                this.turnLeftBackward(nbrSendingMsg);
                break;
            default:
                console.log("Invalid movement command");
                break;
        }
    }

    private sendMsg(nbrSendingMsg: number, data: Twist) {
        const msg = new Message(data)
        for(let _ = 0; _ < nbrSendingMsg; _++) {
            this.publisherMouvement.publish(msg)
            delay(250)
        }
    }

    private moveForward(nbrSendingMsg: number) {
        const data: Twist = {
            linear: {
                x: 1
            }
        } 
        this.sendMsg(nbrSendingMsg, data)
    }   
    
    private moveBackward(nbrSendingMsg: number) {
        const data: Twist = {
            linear: {
                x: -1
            }
        } 
        this.sendMsg(nbrSendingMsg, data)
    }

    private turnLeftForward(nbrSendingMsg: number) {
        const data: Twist = {
            linear: {
                x: 1
            },
            angular: {
                z: -1
            }
        } 
        this.sendMsg(nbrSendingMsg, data)
    }

    private turnRightForward(nbrSendingMsg: number) {
        const data: Twist = {
            linear: {
                x: 1
            },
            angular: {
                z: 1
            }
        } 
        this.sendMsg(nbrSendingMsg, data)
    }

    private turnRightBackward(nbrSendingMsg: number) {
        const data: Twist = {
            linear: {
                x: -1
            },
            angular: {
                z: 1
            }
        } 
        this.sendMsg(nbrSendingMsg, data)
    }

    private turnLeftBackward(nbrSendingMsg: number) {
        const data: Twist = {
            linear: {
                x: -1
            },
            angular: {
                z: -1
            }
        } 
        this.sendMsg(nbrSendingMsg, data)
    }

}
