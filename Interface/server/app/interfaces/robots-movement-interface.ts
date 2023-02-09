import Command from "../types/Command";
import RobotTargetType from "../types/RobotType";

export default interface RobotMovement {
    robot: RobotTargetType,
    direction: Command,
    distance: number
}