import { env } from "process";

export const ROS_MASTER_URI = `${env.ROS_MASTER_URI}` || "http://localhost:11311"

export const URL_NODE_SERVER = 'http://limo-node-server:8080/'