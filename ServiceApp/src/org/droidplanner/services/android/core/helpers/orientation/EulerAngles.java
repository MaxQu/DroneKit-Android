package org.droidplanner.services.android.core.helpers.orientation;

/**
 * Created by mzqu on 8/6/2015.
 */
public class EulerAngles extends Vector3f{
    public static final float R2D=180.0f/(float) Math.PI;
    public static final float D2R=(float) Math.PI/180.0f;

    //override the constructor
    public EulerAngles(float x, float y, float z) {
        super(x,y,z);
    }

    public EulerAngles() {
        super();
    }

    /**
     * Copy constructor
     */
    public EulerAngles(EulerAngles vector) {
        this.points[0] = vector.points[0];
        this.points[1] = vector.points[1];
        this.points[2] = vector.points[2];
    }

    public EulerAngles(Vector3f vector) {
        super(vector);
    }

    public EulerAngles(float[] v) {
        super(v);
    }

    //end of constructor


    public float getYaw() {
        return points[0];
    }

    public float getPitch() {
        return points[1];
    }

    public float getRoll() {
        return points[2];
    }

    public void setZeroRotation() {
        reset();
    }

    public void set(EulerAngles euler){
        super.clone(euler);
    }
    //////////////////////////////////////////////////////////////
    ////////////////////////Quaternion//////////////////////////
    /////////////////////////////////////////////////////////////

    public void setFromQuaternion(Quaternion rv) {
//		currently, only good for theta between -pi/2 and pi/2, and switch to 3/2pi to pi/2
//		rv has to be the rotation quaternion from body to earth
        rv.toEulerAngles(this);
    }



    public void toQuaternion(Quaternion q) {
        q.setFromEulerAngles(this);
    }

    public Quaternion getQuaternion() {

        Quaternion q= new Quaternion();
        toQuaternion(q);
        return q;
    }


    ///////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////Rotation Matrix/////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////
    public void setFromRotationMatrix(RotationMatrix rm){
//		currently, only good for theta between -pi/2 and pi/2, and switch to 3/2pi to pi/2
//		rm has to be the rotation matrix from body to earth

        rm.toEulerAngles(this);
    }

    public void toRotationMatrix(RotationMatrix rm) {
        //this function get you R b to e, see ref2. page 22, standard notation
        // R b to e [xb in e, yb in e, zb in e]

        rm.setFromEulerAngles(this);
    }

    public RotationMatrix getRotationMatrix() {
        //this function get you R b to e, see ref2. page 22, standard notation
        // R b to e [xb in e, yb in e, zb in e]

        RotationMatrix rm = new RotationMatrix();
        toRotationMatrix(rm);
        return rm;
    }

    ////////////////////////////////////////////////////////////////////
    ////////////////////////Axis Angle//////////////////////////////////
    ////////////////////////////////////////////////////////////////////
    public void setFromAxisAngle(AxisAngle n) {
        //NOT TESTED
        //use quaternion as a intermediate step
        n.getQuaternion().toEulerAngles(this);

    }

    public void toAxisAngle(AxisAngle n) {
        //NOT TESTED
        //use quaternion as a intermediate step
        this.getQuaternion().toAxisAngle(n);
    }

    public AxisAngle getAxisAngle() {
        //NOT TESTED
        //use quaternion as a intermediate step
        AxisAngle n = new AxisAngle();
        this.getQuaternion().toAxisAngle(n);
        return n;
    }

    ////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////

    public void rotateVQ(Vector3f r1, Vector3f r2) {
        //rotate vector through Quaternion
        this.getQuaternion().rotateV(r1,r2);
    }


    public Vector3f rotateVQ(Vector3f r1) {
        //rotate vector through Quaternion
        return this.getQuaternion().rotateV(r1);
    }

    ////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////

    public void toDiffEulerAnglesToTarget(EulerAngles targetE,EulerAngles diffE) {
        //use quaternion as an intermediate step
        getQuaternion().getDiffQuaternionToTarget(targetE.getQuaternion()).toEulerAngles(diffE);
    }

    public EulerAngles getDiffEulerAnglesToTarget(EulerAngles targetE) {
        //use quaternion as an intermediate step
        EulerAngles diffE = new EulerAngles();
        toDiffEulerAnglesToTarget(targetE, diffE);
        return diffE;
    }

    /////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////

    public Vector3f getDegree() {
        Vector3f euler = new Vector3f();
        euler.points[0]=points[0]*R2D;
        euler.points[1]=points[1]*R2D;
        euler.points[2]=points[2]*R2D;

        return euler;
    }







}
