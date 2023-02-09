// COMMAND TO SEND TO ROBOT TO MOVE

import Command from "../types/Command"

export const DISTANCE_MOVEMENT = {
    TOO_FAR: 20,
    FAR_AWAY: 15,
    FAR: 10,
    CLOSE: 5,
    VERY_CLOSE: 2,
    TOO_CLOSE: 1
}

export const DIRECTION_MOVEMENT = {
    FORWARD: "forward" as Command,
    BACKWARD: "backward" as Command,
    LEFT_FORWARD: "left-forward" as Command,
    RIGHT_FORWARD: "right-forward" as Command,
    LEFT_BACKWARD: "left-backward" as Command,
    RIGHT_BACKWARD: "right-backward"as Command
}

