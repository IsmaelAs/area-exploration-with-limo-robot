import Command from "@app/types/Command";

export default interface OnRobotMovement {
    direction: Command,
    distance: number
}