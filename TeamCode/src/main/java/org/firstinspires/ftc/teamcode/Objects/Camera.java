package org.firstinspires.ftc.teamcode.Objects;

public class Camera extends Equipment {
    //////////////////////////////
    // PROPERTIES
    //////////////////////////////
    // This guy inherits the Robot it is equipped to.
    // The robot in-turn carries a map...

    ///////////////////////////////
    // CONSTRUCTOR(S)
    ///////////////////////////////

    public Camera(){
        super();
    }

    ////////////////////////////////
    // METHODS
    ////////////////////////////////
    public Artifact findArtifact() {
        // Let's assume the Camera tells us it is at the Left Hash position.
        // We will ask the camera's robot to check it's map and get that position.
        Position ArtifactPos = this.getRobot().getMap().getArtHashLeft();
        return new Artifact(ArtifactPos);
    }

}