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

    private gridClient1: GridClient = { };

    private gridClient2: GridClient = { };

    private gridClientMerged: GridClient = { };

    private viewer1: MapViewer = { 'scene': undefined };

    private viewer2: MapViewer = { 'scene': undefined };

    private viewerMerged: MapViewer = { 'scene': undefined };

    private ros1: Ros;

    private ros2: Ros;

    private mapsCreated = false;

    mapsVisible: boolean = false;

    

    constructor (private ipHandler: IpHandlerService) {

        this.ros1 = new Ros({
            'url': `ws://${ipHandler.ipAddressLimo1}:9090`
        });

        this.ros2 = new Ros({
            'url': `ws://${ipHandler.ipAddressLimo2}:9090`
        });

    }

    init () {

        this.swapVisible();

        if (this.mapsCreated) return;
        console.log('creerrrr')

        // Create the main viewer 1.
        this.viewer1 = new ROS3D.Viewer({
            'background': '#7e7e7e',
            'divID': 'map1',
            'width': 800,
            'height': 600,
            'antialias': true
        });

        // Setup the map client 1.
        this.gridClient1 = new ROS3D.OccupancyGridClient({
            'ros': this.ros1,
            'topic': environment.IS_SIMULATION ? '/limo1/map' : '/map',
            'rootObject': this.viewer1.scene,
            'continuous': true
        });

        // Create the main viewer 2.
        this.viewer2 = new ROS3D.Viewer({
            'background': '#7e7e7e',
            'divID': 'map2',
            'width': 800,
            'height': 600,
            'antialias': true
        });

        // Setup the map client 2.
        this.gridClient2 = new ROS3D.OccupancyGridClient({
            'ros': this.ros2,
            'topic': environment.IS_SIMULATION ? '/limo2/map' : 'map',
            'rootObject': this.viewer2.scene,
            'continuous': true
        });


        // Create the Merged Map viewer 2.
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
            'topic': environment.IS_SIMULATION ? '/map_merge/x_merged_map' : '/map_merged/map',
            'rootObject': this.viewerMerged.scene,
            'continuous': true
        });

        this.mapsCreated = true;
    }

    swapVisible() {
        this.mapsVisible = !this.mapsVisible;
    }


}
