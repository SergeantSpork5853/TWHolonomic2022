// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class RPLidar {
    private NetworkTable table;
    private NetworkTableEntry lidarModel;
    private NetworkTableEntry firmwareVersion;
    private NetworkTableEntry hardwareVersion;
    private NetworkTableEntry serialNumber;
    private NetworkTableEntry isRunning;
    private NetworkTableEntry scanIndex;
    private NetworkTableEntry objectCount;
    private NetworkTableEntry objectAngleBearings;
    private NetworkTableEntry objectDistances;
    private NetworkTableEntry objectWidths;
    private NetworkTableEntry minDetectDistance;
    private NetworkTableEntry maxDetectDistance;
    private NetworkTableEntry objectBackroundDelta;
    private NetworkTableEntry angleDetectWindow;
    private NetworkTableEntry maxObjectWidth;
    private NetworkTableEntry minObjectWidth;
    private NetworkTableEntry lidarScannerOrientation;

    public RPLidar() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();

        NetworkTable datatable = inst.getTable("rp_lidar");

        lidarModel = table.getEntry("lidar_model");
        firmwareVersion = table.getEntry("firmware_ver");
        hardwareVersion = table.getEntry("hardware_ver");
        isRunning = datatable.getEntry("running");
        scanIndex = datatable.getEntry("scan_index");
        objectCount = datatable.getEntry("num_objects");
        objectAngleBearings = datatable.getEntry("object_bearings");
        objectDistances = table.getEntry("object_distances");
        objectWidths = table.getEntry("object_widths");
        minDetectDistance = table.getEntry("min_detect_distance");
        maxDetectDistance = table.getEntry("max_detect_distance");
        objectBackroundDelta = table.getEntry("object_bg_delta");
        angleDetectWindow = table.getEntry("angle_detect_window");
        maxObjectWidth = table.getEntry("max_object_width");
        minObjectWidth = table.getEntry("min_object_width");
        lidarScannerOrientation = table.getEntry("scanner_orientation");
        inst.startClientTeam(1405);
        inst.startDSClient();
    }

    public void publishData() {

        if (objectCount.getNumber(0).intValue() > 0) {
            int objects = objectCount.getNumber(0).intValue();
            Number[] defaultvalue = new Number[0];
            Number[] bearings = objectAngleBearings.getNumberArray(defaultvalue);

            for (int i = 0; i < objects; i++) {
                System.out.println("Bearings " + bearings[i]);
            }

        }

    }
}
