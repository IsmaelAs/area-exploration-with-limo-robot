import delay from 'delay'
import {Topic, Ros, Message} from 'roslib'
import Twist from '../../../interfaces/Twist'
import { ROS_MASTER_URI } from '../../../constants/url'
import Command from "../../../types/Command"

export class NodeMouvement {
    private name: String = "Node Mouvement"

    private publisherMouvement: Topic
    private ros: Ros
    private nulVelocityMsg: Message

    // Connect the node to the Limo
    initNodeMouvement(): void {
        this.ros = new Ros({ url: ROS_MASTER_URI })


        this.nulVelocityMsg = new Message({
            linear: {
                x: 0,
                y: 0, 
                z: 0
            },
            angular: {
                x: 0,
                y: 0,
                z: 0
            }
        }) 

        // Console error when error
        this.ros.on('error', (err: Error) => {
            console.error(`${this.name} : ${err.stack}`)
        })

        // Wait for ROS to connect to the bridge
        this.ros.on('connection', () => {
            console.log(`${this.name} : ROS connected`);
        });

        // initialise publisher
        this.publisherMouvement = new Topic({
            ros: this.ros,
            name: "cmd_vel",
            messageType: "geometry_msgs/Twist",                
            queue_size: 10
        })

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
    async move(command: Command, nbrSendingMsg = 5): Promise<void> {
        console.log(`${this.name} : Moving ${command}`);

        switch (command) {
            case "forward":
                await this.moveForward(nbrSendingMsg);
                break;
            case "backward":
                await this.moveBackward(nbrSendingMsg);
                break;
            case "left-forward":
                await this.turnLeftForward(nbrSendingMsg);
                break;
            case "right-forward":
                await this.turnRightForward(nbrSendingMsg);
                break;
            case "right-backward":
                await this.turnRightBackward(nbrSendingMsg);
                break;
            case "left-forward":
                await this.turnLeftBackward(nbrSendingMsg);
                break;
            default:
                console.log("Invalid movement command");
                break;
        }
    }

    private async sendMsg(nbrSendingMsg: number, data: Twist) {
        const msg = new Message(data)
        for(let _ = 0; _ < nbrSendingMsg; _++) {
            this.publisherMouvement.publish(msg)
            await delay(250)
        }
        this.publisherMouvement.publish(this.nulVelocityMsg)
    }

    private async moveForward(nbrSendingMsg: number) {
        const data: Twist = {
            linear: {
                x: 1
            }
        } 
        await this.sendMsg(nbrSendingMsg, data)
    }   
    
    private async moveBackward(nbrSendingMsg: number) {
        const data: Twist = {
            linear: {
                x: -1
            }
        } 
        await this.sendMsg(nbrSendingMsg, data)
    }

    private async turnLeftForward(nbrSendingMsg: number) {
        const data: Twist = {
            linear: {
                x: 1,

            },
            angular: {
                z: -1
            }
        } 
        await this.sendMsg(nbrSendingMsg, data)
    }

    private async turnRightForward(nbrSendingMsg: number) {
        const data: Twist = {
            linear: {
                x: 1
            },
            angular: {
                z: 1
            }
        } 
        await this.sendMsg(nbrSendingMsg, data)
    }

    private async turnRightBackward(nbrSendingMsg: number) {
        const data: Twist = {
            linear: {
                x: -1
            },
            angular: {
                z: 1
            }
        } 
        await this.sendMsg(nbrSendingMsg, data)
    }

    private async turnLeftBackward(nbrSendingMsg: number) {
        const data: Twist = {
            linear: {
                x: -1
            },
            angular: {
                z: -1
            }
        } 
        await this.sendMsg(nbrSendingMsg, data)
    }

}
