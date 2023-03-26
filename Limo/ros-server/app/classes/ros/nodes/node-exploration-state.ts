import { BRIDGE_URI } from '../../../constants/url';
import {Message, Ros, Topic} from 'roslib';
import Bool from '../../../types/Bool';

export class NodeExplorationState {
  private ros: Ros;

  private publisherExplorationState: Topic;

  private name = 'Node Exploration State';


  initNodeExplorationState() {
    this.ros = new Ros({url: BRIDGE_URI});

    // Console error when error
    this.ros.on('error', (err: Error) => {
      console.error(`${this.name} : ${err.message}`);
    });

    this.ros.on('connection', () => {
      console.log(`${this.name} : ROS connected`);

      this.publisherExplorationState = new Topic({
        ros: this.ros,
        name: 'exploration_state',
        messageType: 'std_msgs/Bool',
        queue_size: 10,
      });
    });
  }

  sendMessage(msg: Bool) {
    const msgToSend = new Message(msg);
    this.publisherExplorationState.publish(msgToSend);
  }

  closeNodeExplorationState() {
    if (this.ros) this.ros.close();
  }
}
