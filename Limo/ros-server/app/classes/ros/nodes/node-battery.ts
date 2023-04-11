import { Ros, Topic } from 'roslib';
import { BRIDGE_URI } from '../../../constants/url';

export class NodeBattery {
  private ros: Ros;

  private batterySubscriber: Topic;

  private data: {percentage: number};

  private name = 'Node Battery';

  isLowBattery = false;

  initNodeBattery() {
    this.ros = new Ros({ url: BRIDGE_URI });

    this.ros.on('error', (err: Error) => {
      console.error(`${this.name} : ${err.message}`);
    });

    // Wait for ROS to connect to the bridge
    this.ros.on('connection', () => {
      console.log(`${this.name} : ROS connected`);
      this.batterySubscriber = new Topic({
        ros: this.ros,
        name: '/limo_status',
        messageType: 'limo_base/LimoStatus',
      });
      this.batterySubscriber.subscribe(this.callBack.bind(this));
    });
  }

  private callBack(message: { battery_voltage: number; }): void {
    const minVoltage = 8.25;
    const maxVoltage = 12.6;
    const batteryVoltage = message.battery_voltage;
  
    // Calculate the battery percentage
    const percentage = ((batteryVoltage - minVoltage) / (maxVoltage - minVoltage)) * 100;
  
    // Log the battery level
    console.log('This is the battery level:', percentage);
  
    // Check if the battery is below 30%
    if (percentage < 30) {
      console.log('Battery level is below 30%');
      this.isLowBattery = true;
    } else {
      this.isLowBattery = false;
    }
  
    // Update the data object
    this.data = { percentage };
  }
  


  getData(): {percentage: number} {
    return this.data;
  }

  closeNodeBattery() {
    if (this.ros) this.ros.close();
  }
}
