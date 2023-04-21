import { HttpClient } from '@angular/common/http';
import { Injectable } from '@angular/core';
import { Observable, of } from 'rxjs';
import { catchError } from 'rxjs/operators';
import { environment } from '../../environments/environment';
import MissionInfos from '../interfaces/mission-infos';

@Injectable({
    'providedIn': 'root'
})
export class CommunicationService {
    private readonly baseUrl: string;

    constructor (private readonly http: HttpClient) {
        this.baseUrl = `http://${environment.BACKEND_IP}:9330`;
    }

    getMissions (): Observable<MissionInfos[]> {
        return this.http.get<MissionInfos[]>(`${this.baseUrl}/history/missions`).pipe(catchError(this.handleError<MissionInfos[]>('getMissions')));
    }

    getTests (): Observable<any> {
        return this.http.get<any>(`${this.baseUrl}/tests`).pipe(catchError(this.handleError<any>('getTests')));
    }

    // eslint-disable-next-line class-methods-use-this
    private handleError<T> (request: string, result?: T): (error: Error) => Observable<T> {
        return () => of(result as T);
    }
}
