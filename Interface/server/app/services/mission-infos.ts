/* eslint-disable no-magic-numbers */
import { Server as SocketServer } from 'socket.io';
import DistanceInfo from '../interfaces/distance-info';
import { DB_MISSION_INFOS_COLLECTION } from '../constants/data-base-constants';
import { DataBaseHandler } from './data-base-handler';

export class MissionInfos {
  private missionStart: Date;

  private currentDate = '';

  private currentHour = '';

  private duration = '';

  private dataBaseHandler: DataBaseHandler;

  private io: SocketServer;

  constructor(io: SocketServer) {
    this.dataBaseHandler = new DataBaseHandler();
    this.io = io;
  }

  onMissionStart(): void {
    this.missionStart = new Date();
    // Create a new date object with only the year, month, and day
    const date: Date = new Date(this.missionStart.getFullYear(), this.missionStart.getMonth(), this.missionStart.getDate());
    this.currentDate = date.toISOString().substring(0, 10);

    const hour: number = this.missionStart.getHours();
    const minute: number = this.missionStart.getMinutes();

    this.currentHour = `${hour}h${minute < 10 ? '0' : ''}${minute}`;
  }

  onMissionEnd(): void {
    const newDate = new Date();
    const durationMs: number = newDate.getTime() - this.missionStart.getTime();

    // Converting the duration to minutes and seconds
    const durationMin: number = Math.floor(durationMs / (1000 * 60));
    const durationSec: number = Math.floor(durationMs % (1000 * 60) / 1000);
    this.duration = `${durationMin} min ${durationSec} s`;
  }

  async saveTotalDistance(data: DistanceInfo): Promise<void> {
    console.log(data);
    const {totalDistance} = data;
    await this.dataBaseHandler.insert(DB_MISSION_INFOS_COLLECTION, {
      date: this.currentDate,
      heure: this.currentHour,
      duration: this.duration,
      limoId: data.limoId,
      totalDistance,
    });
    this.io.emit('refreshDb');
  }
}
