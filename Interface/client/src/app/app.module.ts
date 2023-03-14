import { NgModule } from '@angular/core';
import { BrowserModule } from '@angular/platform-browser';

import { AppRoutingModule } from './app-routing.module';
import { AppComponent } from './app.component';

import {MatSelectModule} from '@angular/material/select';
import { BrowserAnimationsModule } from '@angular/platform-browser/animations';
import { FormsModule } from '@angular/forms';
import { LogsDialogComponent } from './dialogs/logs-dialog/logs-dialog.component';
import { MatDialogModule } from '@angular/material/dialog';
import { LogActionComponent } from './components/log-action/log-action.component';
import { ActionButtonsComponent } from './components/action-buttons/action-buttons.component';
import { RobotsActionsComponent } from './components/robots-actions/robots-actions.component';


@NgModule({
    'declarations': [
        AppComponent,
        LogsDialogComponent,
        LogActionComponent,
        ActionButtonsComponent,
        RobotsActionsComponent

    ],
    'imports': [
        BrowserModule,
        AppRoutingModule,
        MatSelectModule,
        BrowserAnimationsModule,
        FormsModule,
        MatDialogModule
    ],
    'providers': [],
    'bootstrap': [AppComponent]
})
export class AppModule { }

