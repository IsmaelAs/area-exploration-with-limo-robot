import Point from './Point';
import Quaternion from './Quaternion';

type Pose = {
    position: Point,
    orientation: Quaternion
}

export default Pose;
