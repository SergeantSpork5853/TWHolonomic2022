// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class JetsonRpLidar {
    private NetworkTableEntry x;
    private NetworkTableEntry y;
    private NetworkTableEntry theta;

    public JetsonRpLidar() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();

        NetworkTable datatable = inst.getTable("TestNetTable");

        x = datatable.getEntry("x");
        y = datatable.getEntry("y");
        theta = datatable.getEntry("theta");
        inst.startClientTeam(1405);
        inst.startDSClient();

    }

    public void publishData() {
        // double rotation = theta.getNumber(0).doubleValue();

        // System.out.println("Theta " + Math.toDegrees(getTheta()));

        // System.out.println("y " + getY());

        SmartDashboard.putNumber("theta", Math.toDegrees(getTheta()));
        SmartDashboard.putNumber("xDistance", getX());
        SmartDashboard.putNumber("yDistance", getY());
    }

    public double getX() {
        return x.getNumber(0).doubleValue();
    }

    public double getY() {
        return y.getNumber(0).doubleValue();
    }

    public double getTheta() {
        return theta.getNumber(0).doubleValue();
    }
}
