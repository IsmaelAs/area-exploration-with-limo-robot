import { NgModule } from '@angular/core';
import { BrowserModule } from '@angular/platform-browser';

import { AppRoutingModule } from './app-routing.module';
import { AppComponent } from './app.component';

import { MatIconModule } from '@angular/material/icon'
import {MatSelectModule} from '@angular/material/select';
import { BrowserAnimationsModule } from '@angular/platform-browser/animations';
import { FormsModule } from '@angular/forms';
import { LogsDialogComponent } from './dialogs/logs-dialog/logs-dialog.component';
import { MatDialogModule } from '@angular/material/dialog';
import { LogActionComponent } from './components/log-action/log-action.component';
import { ActionButtonsComponent } from './components/action-buttons/action-buttons.component';
import { RobotsActionsComponent } from './components/robots-actions/robots-actions.component';
import { IpLimoHandlerComponent } from './components/ip-limo-handler/ip-limo-handler.component';
import { MapViewerComponent } from './components/map-viewer/map-viewer.component';
import { RobotsStateComponent } from './components/robots-state/robots-state.component';
import { MainPageComponent } from './components/main-page/main-page.component';
import { P2pHandlerComponent } from './components/p2p-handler/p2p-handler.component';

@NgModule({
    'declarations': [
        AppComponent,
        MapViewerComponent,
        LogsDialogComponent,
        LogActionComponent,
        ActionButtonsComponent,
        RobotsActionsComponent,
        IpLimoHandlerComponent,
        RobotsStateComponent,
        MainPageComponent,
        P2pHandlerComponent

    ],
    'imports': [
        BrowserModule,
        AppRoutingModule,
        MatIconModule,
        MatSelectModule,
        BrowserAnimationsModule,
        FormsModule,
        MatDialogModule
    ],
    'providers': [],
    'bootstrap': [AppComponent]
})
export class AppModule { }

