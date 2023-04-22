import { RobotsActionsComponent } from '../robots-actions/robots-actions.component';
import { RobotsStateComponent } from '../robots-state/robots-state.component';
import { LogActionComponent } from '../log-action/log-action.component';
import { P2pHandlerComponent } from '../p2p-handler/p2p-handler.component';
import { MapViewerComponent } from '../map-viewer/map-viewer.component';
import { MainPageComponent } from './main-page.component';
import { ComponentFixture, TestBed } from '@angular/core/testing';
import { MatDialogModule } from '@angular/material/dialog';

describe('MainPageComponent', () => {
  let component: MainPageComponent;
  let fixture: ComponentFixture<MainPageComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      imports: [MatDialogModule],
      declarations: [ 
        MainPageComponent,
        RobotsActionsComponent,
        RobotsStateComponent,
        LogActionComponent,
        P2pHandlerComponent,
        MapViewerComponent
      ]
    })
    .compileComponents();
  });

  beforeEach(() => {
    fixture = TestBed.createComponent(MainPageComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });

  // Ajouter d'autres tests ici si nécessaire
});
