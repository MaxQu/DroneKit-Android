package org.droidplanner.services.android.core.helpers.orientation;

/**
 * Created by mzqu on 8/6/2015.
 */
public class AxisAngle extends Vector4f{
    public static final float R2D=180.0f/(float) Math.PI;
    public static final float D2R=(float) Math.PI/180.0f;
    //override the constructor
    public AxisAngle(float x, float y, float z, float w) {
        super(x,y,z,w);
    }

    public AxisAngle() {
        super();
    }

    /**
     * Copy constructor
     */
    public AxisAngle(AxisAngle vector) {
        this.points[0] = vector.points[0];
        this.points[1] = vector.points[1];
        this.points[2] = vector.points[2];
    }

    public AxisAngle(float[] v) {
        super(v);
    }

    public AxisAngle(Vector3f v, float w) {
        super(v,w);
    }

    //end of constructor


    public void setZeroRotation() {
        reset();
    }

    public void set(AxisAngle euler){
        super.copyVec4(euler);
    }

    ///////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////Rotation Matrix///////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////

    public void setFromMatrix(RotationMatrix rm) {
        //see ref2. 32, NOT TESTED
        rm.toAxisAngle(this);
    }
    public void toRotationMatrix(RotationMatrix rm) {
        //n has to be a valid axis angle representation, the first three elements are unit axis representation
        //the last element is theta valid from 0 to pi.
        rm.setFromAxisAngle(this);
    }

    public RotationMatrix getRotationMatrix() {
        //n has to be a valid axis angle representation, the first three elements are unit axis representation
        //the last element is theta valid from 0 to pi.

        RotationMatrix rm = new RotationMatrix();
        toRotationMatrix(rm);
        return rm;
    }

    /////////////////////////////////////////////////////////////////
    ////////////////////////////Quaternion///////////////////////////
    ////////////////////////////////////////////////////////////////

    public void setFromQuaternion(Quaternion q) {
        //NOT TESTED
        q.toAxisAngle(this);
    }

    public void toQuaternion(Quaternion q) {
        //n has to be a valid axis angle representation, the first three elements are unit axis representation
        q.setFromAxisAngle(this);
    }

    public Quaternion getQuaternion() {
        //n has to be a valid axis angle representation, the first three elements are unit axis representation
        Quaternion q = new Quaternion();
        toQuaternion(q);
        return q;
    }

    /////////////////////////////////////////////////////////////////
    /////////////////////////Euler Angles////////////////////////////
    ///////////////////////////////////////////////////////////////////

    public void setFromEulerAngles(EulerAngles euler) {
        euler.getQuaternion().toAxisAngle(this);
    }

    public void toEulerAngles(EulerAngles euler) {
        this.getQuaternion().toEulerAngles(euler);
    }

    public EulerAngles getEulerAngles() {
        EulerAngles euler = new EulerAngles();
        this.getQuaternion().toEulerAngles(euler);
        return euler;
    }



    public void rotateVQ(Vector3f r1, Vector3f r2) {
        //rotate vector through Quaternion
        this.getQuaternion().rotateV(r1, r2);
    }


    public Vector3f rotateVQ(Vector3f r1) {
        //rotate vector through Quaternion
        return this.getQuaternion().rotateV(r1);
    }

    ////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////

    public void toDiffAxisAngleToTarget(AxisAngle targetA,AxisAngle diffA) {
        //use quaternion as an intermediate step
        getQuaternion().getDiffQuaternionToTarget(targetA.getQuaternion()).toAxisAngle(diffA);
    }

    public AxisAngle getDiffAxisAngleToTarget(AxisAngle targetA) {
        //use quaternion as an intermediate step
        AxisAngle diffA = new AxisAngle();
        toDiffAxisAngleToTarget(targetA, diffA);
        return diffA;
    }

    /////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////
    public Vector4f getDegree() {
        Vector4f axisAngle = new Vector4f();
        axisAngle.points[0]=points[0];
        axisAngle.points[1]=points[1];
        axisAngle.points[2]=points[2];
        axisAngle.points[3]=points[3]*R2D;

        return axisAngle;
    }
}
