import { Ros, Topic } from 'roslib'
import { BRIDGE_URI } from '../../../constants/url'
import Odometry from '@app/types/Odometry'

export class NodePosition {
    private ros: Ros
    private subscriberMovement: Topic
    private data: Odometry


    initNodePosition() {
        this.ros = new Ros({url: BRIDGE_URI})
        this.subscriberMovement = new Topic({
            ros: this.ros, 
            name: "odom",
            messageType: "nav_msgs/Odometry",
            queue_size: 10
        })

        this.subscriberMovement.subscribe(this.callBack.bind(this))
    }


    private callBack(data: Odometry): void {
        this.data = data
    }

    getData(): Odometry {
        return this.data
    }
}