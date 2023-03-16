import Twist from './Twist';

type TwistWithCovariance = {
    twist: Twist,
    covariance: number[]
}

export default TwistWithCovariance;
