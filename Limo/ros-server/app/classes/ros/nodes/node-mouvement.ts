import { ROS_MASTER_URI } from '../../../constants/url'
import { BehaviorSubject } from 'rxjs'

const ros = require('rosnodejs')

export class NodeMouvement {

    publisherMouvement: unknown
    subscriberMouvement: unknown

    valuePassor: BehaviorSubject<unknown>
    constructor () {
        console.log("ros mange tes grand mort fdp ster uri" , ROS_MASTER_URI);
        
        ros.initNode('mouvement_node', {rosMasterUri: ROS_MASTER_URI}).then(() => {
            console.log('allo')
            // const nh = ros.nh
            // this.publisherMouvement = nh.advertise('topic', 'type')
            // this.subscriberMouvement = nh.subscribe('topic', 'type', (msg: String) => {
            //     this.valuePassor.next(msg)
            // })
        })
    }

    publish() {
    }

    subscribe() {
        this.valuePassor.asObservable()
    }
}
