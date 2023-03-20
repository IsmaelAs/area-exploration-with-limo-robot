import { TestBed, ComponentFixture } from '@angular/core/testing';
import { RouterTestingModule } from '@angular/router/testing';
import { AppComponent } from './app.component';
import { AppRoutingModule } from './app-routing.module';
import { IpLimoHandlerComponent } from './components/ip-limo-handler/ip-limo-handler.component';
import { LogActionComponent } from './components/log-action/log-action.component';
import { RobotsActionsComponent } from './components/robots-actions/robots-actions.component';
import { FormsModule } from '@angular/forms';

describe('AppComponent', () => {
  let component: AppComponent
  let fixture: ComponentFixture<AppComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      imports: [
        RouterTestingModule,
        AppRoutingModule,
        FormsModule
      ],
      declarations: [
        AppComponent,
        IpLimoHandlerComponent,
        LogActionComponent,
        RobotsActionsComponent
      ],
    }).compileComponents();

    fixture = TestBed.createComponent(AppComponent)
    component = fixture.componentInstance
    fixture.detectChanges()
  });

  it('should create the app', () => {
    const fixture = TestBed.createComponent(AppComponent);
    const app = fixture.componentInstance;
    expect(app).toBeTruthy();
  });

  it("should isIpSet with the event", () => {
    component.isIpSet = false

    component.setIsIpSet({ isIpSet: true })

    expect(component.isIpSet).toBeTruthy()
  })

});
