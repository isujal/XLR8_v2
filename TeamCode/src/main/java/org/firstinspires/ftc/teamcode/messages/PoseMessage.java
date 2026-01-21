package org.firstinspires.ftc.teamcode.messages;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

public final class PoseMessage {
    public long timestamp;
    public double x;
    public double y;
    public double heading;

    public PoseMessage(Pose2d pose) {
        this.timestamp = System.nanoTime();
        this.x = pose.position.x;
        this.y = pose.position.y;
        this.heading = pose.heading.toDouble();
    }


//    public PoseMessage(Pose2d pose) {
//        Vector2d v=new Vector2d(pose.position.x,pose.position.y);
//        this.timestamp = System.nanoTime();
//        this.x = pose.position.x;
//        this.y = pose.position.y;
//        this.heading = pose.heading.toDouble();
//    }
//
//


}

