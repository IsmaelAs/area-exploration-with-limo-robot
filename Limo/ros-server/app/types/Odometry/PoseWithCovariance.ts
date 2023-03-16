import Pose from './Pose';

type PoseWithCovariance = {
    pose: Pose,
    covariance: number[]
}

export default PoseWithCovariance;
