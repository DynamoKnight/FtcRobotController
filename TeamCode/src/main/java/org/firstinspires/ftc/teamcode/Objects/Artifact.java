package org.firstinspires.ftc.teamcode.Objects;

public class Artifact extends Thing {
    //////////////////////////////
    // PROPERTIES
    //////////////////////////////

    ///////////////////////////////
    // CONSTRUCTOR
    ///////////////////////////////
    public Artifact(int x, int y) {
        super(x, y);
    }

    public Artifact(Position p){
        super();
        this.setPos(p);
    }

    ////////////////////////////////
    // METHODS
    ////////////////////////////////

}