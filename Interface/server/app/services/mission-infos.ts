
import DistanceInfo from '../interfaces/distance-info';
import { DB_MISSION_INFOS_COLLECTION } from '../constants/data-base-constants';
import { DataBaseHandler } from './data-base-handler';

export class MissionInfos {
  private missionStart: Date;

  private missionEnd: Date;

  private currentDate: string;

  private currentHour: string;

  private duration: string;

  private distanceLimo1?: number = 0;

  private distanceLimo2?: number = 0;

  private dataBaseHandler: DataBaseHandler;


  onMissionStart(): void {
    this.missionStart = new Date();
    this.saveMissionDateAndHour();
  }

  onMissionEnd(): void {
    this.missionEnd = new Date();
    this.saveTotalDuration();
  }

  saveMissionDateAndHour(): void {
    // Create a new date object with only the year, month, and day
    const date: Date = new Date(this.missionStart.getFullYear(), this.missionStart.getMonth(), this.missionStart.getDate());
    this.currentDate = date.toISOString().substring(0, 10);

    const hour: number = this.missionStart.getHours();
    const minute: number = this.missionStart.getMinutes();

    this.currentHour = `${hour}h${minute < 10 ? '0' : ''}${minute}`;
  }

  saveTotalDuration(): void {
    const durationMs: number = this.missionEnd.getTime() - this.missionStart.getTime();

    // Converting the duration to minutes and seconds
    const durationMin: number = Math.floor(durationMs / (1000 * 60));
    const durationSec: number = Math.floor(durationMs % (1000 * 60) / 1000);

    this.duration = `${durationMin} min ${durationSec} s`;
  }

  saveTotalDistance(data: DistanceInfo): void {
    if (data.limoId === 1) this.distanceLimo1 = data.distance;
    if (data.limoId === 2) this.distanceLimo2 = data.distance;
    this.insertMissionsInfos();
  }

  async insertMissionsInfos():Promise<void> {
    await this.dataBaseHandler.insert(DB_MISSION_INFOS_COLLECTION, {
      date: this.currentDate,
      heure: this.currentHour,
      durée: this.duration,
      distanceLimo1: this.distanceLimo1,
      distanceLimo2: this.distanceLimo2,
    }
    );
  }

  /*
   * Async getAllMissionsInfos():Promise<void> {
   *   const data = await this.dataBaseHandler.find(DB_MISSION_INFOS_COLLECTION);
   *   this.socket.emit('send-mission-infos', data);
   * }
   */
}
