import { TestBed } from '@angular/core/testing';

import { IpHandlerService } from './ip-handler.service';

describe('IpHandlerService', () => {
  let service: IpHandlerService;

  beforeEach(() => {
    TestBed.configureTestingModule({});
    service = TestBed.inject(IpHandlerService);
  });

  it('should be created', () => {
    expect(service).toBeTruthy();
  });
});
