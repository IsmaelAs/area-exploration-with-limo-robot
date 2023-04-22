import { BRIDGE_URI } from '../../../constants/url';
import { Message, Ros, Topic } from 'roslib';
import Bool from '../../../types/Bool';

export class NodeReturnToBase {
  private ros: Ros;

  private publisherReturnToBase: Topic;

  private name = 'Node Return To Base';


  initNodeReturnToBase() {
    this.ros = new Ros({ url: BRIDGE_URI });

    // Console error when error
    this.ros.on('error', (err: Error) => {
      console.error(`${this.name} : ${err.message}`);
    });

    this.ros.on('connection', () => {
      console.log(`${this.name} : ROS connected`);

      this.publisherReturnToBase = new Topic({
        ros: this.ros,
        name: process.env.IS_SIMULATION ? `limo${process.env.LIMO_ID}/return_to_base` : 'return_to_base',
        messageType: 'std_msgs/Bool',
        queue_size: 10,
      });
    });
  }

  sendMessage(msg: Bool) {
    const msgToSend = new Message(msg);
    this.publisherReturnToBase.publish(msgToSend);
  }

  closeNodeReturnToBase() {
    if (this.ros) this.ros.close();
  }
}
