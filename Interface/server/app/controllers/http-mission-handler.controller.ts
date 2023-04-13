import { Request, Response, Router } from 'express';
import { DataBaseHandler } from '../services/data-base-handler';
import { Service } from 'typedi';
import { DB_MISSION_INFOS_COLLECTION } from '../constants/data-base-constants';

@Service()
export class HttpMissionHandler {
  router: Router;

  private dataBaseHandler: DataBaseHandler;

  constructor() {
    this.configureRouter();
    this.dataBaseHandler = new DataBaseHandler();
  }

  private configureRouter(): void {
    this.router = Router();
    this.router.get('/missions', (req: Request, res: Response) => {
      this.dataBaseHandler.find(DB_MISSION_INFOS_COLLECTION).then((missions) => {
        res.send(missions);
      });
    });
  }
}
