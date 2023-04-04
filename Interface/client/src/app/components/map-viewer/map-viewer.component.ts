import { Component } from '@angular/core';
const ROS3D = require('ros3d');
import ROSLIB, { Ros, TFClient, Transform } from 'roslib';
import { IpHandlerService } from 'src/app/services/ip-handler.service';
import { GridClient, MapViewer, UrdfClient } from 'src/app/types/ros3d';
import { environment } from 'src/environments/environment';
@Component({
    'selector': 'app-map-viewer',
    'templateUrl': './map-viewer.component.html',
    'styleUrls': ['./map-viewer.component.scss']
})

export class MapViewerComponent {

    private urdfClient: UrdfClient = { };

    private tfClient: TFClient;

    private gridClient1: GridClient = { };

    private gridClient2: GridClient = { };

    private gridClientMerged: GridClient = { };

    private viewer1: MapViewer = { 'scene': undefined };

    private viewer2: MapViewer = { 'scene': undefined };

    private viewerMerged: MapViewer = { 'scene': undefined };

    private ros1: Ros;

    private ros2: Ros;

    constructor (private ipHandler: IpHandlerService) {

        this.ros1 = new Ros({
            'url': `ws://${ipHandler.ipAddressLimo1}:9090`
        });

        this.ros2 = new Ros({
            'url': `ws://${ipHandler.ipAddressLimo2}:9090`
        });

        /*
         * Setup a client to listen to TFs.
         * A TF Client that listens to TFs from tf2_web_republisher
         */
        this.tfClient = new ROSLIB.TFClient({
            'ros': this.ros1,
            'fixedFrame': 'world',
            'angularThres': 0.01,
            'transThres': 0.01,
            'rate': 10.0
        });


    }

    init () {

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


        /*
         * Setup the URDF client.
         * create ROS3D.UrdfClient
         * path: provide the source of urdf，package://localPath or meshed，: package:///body/meshed/body.dae
         * or using http://, 如 http://localhost:8000/body/meshed/body.dae
         * the loader will use robot_description as URDF model
         * loader can use to kinds of Collada Loader， PR2 robot ussed ColladalLoader2, other robot model ColladaLoader THREE.js(ROS3D.COLLADA_LOADER)
         */
        this.urdfClient = new ROS3D.UrdfClient({
            'ros': this.ros1,
            'tfClient': this.tfClient,
            'path': 'iiwa_stack',
            'rootObject': this.viewer1.scene,
            'loader': ROS3D.COLLADA_LOADER
        });
    }


}
