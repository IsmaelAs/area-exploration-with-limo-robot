import { Component } from '@angular/core';
const ROS3D = require('ros3d');
import { Ros } from 'roslib';
import { IpHandlerService } from 'src/app/services/ip-handler.service';
import { GridClient, MapViewer } from 'src/app/types/ros3d';
import { environment } from 'src/environments/environment';
@Component({
    'selector': 'app-map-viewer',
    'templateUrl': './map-viewer.component.html',
    'styleUrls': ['./map-viewer.component.scss']
})

export class MapViewerComponent {

    private gridClientMerged: GridClient = { };

    private viewerMerged: MapViewer = { 'scene': undefined };

    private ros1: Ros;


    private mapsCreated = false;

    mapsVisible: boolean = false;

    

    constructor (private ipHandler: IpHandlerService) {

        this.ros1 = new Ros({
            'url': `ws://${ipHandler.ipAddressLimo1}:9090`
        });

    }

    init () {

        this.swapVisible();

        if (this.mapsCreated) return;

        // Create the Merged Map viewer.
        this.viewerMerged = new ROS3D.Viewer({
            'background': '#7e7e7e',
            'divID': 'map3',
            'width': 800,
            'height': 600,
            'antialias': true
        });

        // Setup the Merged map client.
        this.gridClientMerged = new ROS3D.OccupancyGridClient({
            'ros': this.ros1,
            'topic': '/map',
            'rootObject': this.viewerMerged.scene,
            'continuous': true
        });

        this.mapsCreated = true;
    }

    swapVisible() {
        this.mapsVisible = !this.mapsVisible;
    }


}
