import Command from '../types/Command';
import RobotTargetType from '../types/RobotType';

interface RobotMovement {
    robot: RobotTargetType,
    direction: Command,
    distance: number
}

export default RobotMovement;
