import { env } from "process";

export const ROS_MASTER_URI = `${env.ROS_MASTER_URI}` || "http://localhost:11311"