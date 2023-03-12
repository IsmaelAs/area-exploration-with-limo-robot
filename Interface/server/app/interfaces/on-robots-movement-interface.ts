import Command from '@app/types/Command';

interface OnRobotMovement {
    direction: Command,
    distance: number
}

export default OnRobotMovement;

