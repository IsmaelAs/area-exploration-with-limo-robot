import PoseWithCovariance from "./PoseWithCovariance"
import TwistWithCovariance from "./TwistWithCovariance"

type Odometry = {
    pose: PoseWithCovariance
    twist: TwistWithCovariance
}


export default Odometry