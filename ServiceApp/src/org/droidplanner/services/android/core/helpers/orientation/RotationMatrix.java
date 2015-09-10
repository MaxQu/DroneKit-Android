package org.droidplanner.services.android.core.helpers.orientation;

/**
 * Created by mzqu on 8/6/2015.
 */
public class RotationMatrix extends Matrix3by3{

    //The rotation matrix always represents the coordinate transformation from body to earth.
    //In terms of Euler angle, it is = RzRyRx
    //override constructor

    public RotationMatrix(float[] M){
        super(M);
    }

    public RotationMatrix() {
        setIdentity();
    }

    public RotationMatrix(float[][] M) {
        super(M);

    }

    public RotationMatrix(Matrix3by3 M) {
        super(M);
    }

    public RotationMatrix(float[] row1, float[] row2, float[] row3) {
		super(row1,row2,row3);

    }

    public RotationMatrix(Vector3f row1, Vector3f row2, Vector3f row3) {
        super(row1,row2,row3);

    }

    public RotationMatrix(RotationMatrix M){
        System.arraycopy(M.points9by1, 0, points9by1, 0, 9);
        copyToMatrix();
    }

    //end of constructor

    ////////////////////////////////////////////////////
    ////////////////////////////////////////////////////
    //////////////////////////////////////////////////

    public void setIdentity() {
        reset();
        points3by3[0][0]=1;
        points3by3[1][1]=1;
        points3by3[2][2]=1;

        points9by1[0]=1;
        points9by1[4]=1;
        points9by1[8]=1;
    }



    public void set(RotationMatrix M) {
        clone(M);
    }
    ////////////////////////////////////////////////////
    ////////////////////////////////////////////////////
    //////////////////////////////////////////////////

    public void setZeroRotation() {
        setIdentity();
    }

    public Vector3f getXb() {
        //return xb in earth coordinate
        return new Vector3f(points3by3[0][0],points3by3[1][0],points3by3[2][0]);
    }

    public Vector3f getYb() {
        //return yb in earth coordinate
        return new Vector3f(points3by3[0][1],points3by3[1][1],points3by3[2][1]);
    }

    public Vector3f getZb() {
        //return zb in earth coordinate
        return new Vector3f(points3by3[0][2],points3by3[1][2],points3by3[2][2]);
    }

    public Vector3f getXe() {
        //return xe in body coordinate
        return new Vector3f(points3by3[0][0],points3by3[0][1],points3by3[0][2]);
    }

    public Vector3f getYe() {
        //return ye in body coordinate
        return new Vector3f(points3by3[1][0],points3by3[1][1],points3by3[1][2]);
    }

    public Vector3f getZe() {
        //return ze in body coordinate
        return new Vector3f(points3by3[2][0],points3by3[2][1],points3by3[2][2]);
    }

    ///////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////

    public void multiplyMM(RotationMatrix m2, RotationMatrix m3) {
        super.multiplyMM(m2,m3);
    }

    public RotationMatrix multiplyMM(RotationMatrix m2) {
        RotationMatrix m3 = new RotationMatrix();
        super.multiplyMM(m2,m3);
        return m3;
    }

    ////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////
    @Override
    public RotationMatrix getTranspose() {
        RotationMatrix tM= new RotationMatrix(this);
        tM.transpose();
        return tM;
    }
    ///////////////////////////////////////////////////////////////
    ////////////////////Quaternion/////////////////////////////////
    ///////////////////////////////////////////////////////////////

    public void setFromQuaternion(Quaternion q) {
        q.toRotationMatrix(this);
    }


    public void toQuaternion(Quaternion q) {
        //see ref2. 35	NOT TESTED
        q.setFromRotationMatrix(this);
    }

    public Quaternion getQuaternion() {
        //see ref2. 35	NOT TESTED
        Quaternion q = new Quaternion();
        toQuaternion(q);
        return q;
    }

    ///////////////////////////////////////////////////////////////////
    ///////////////////Euler Angles////////////////////////////////////
    ///////////////////////////////////////////////////////////////////
    public void setFromEulerAngles(EulerAngles euler) {
        float cpsi = (float) Math.cos(euler.points[0]);
        float spsi = (float) Math.sin(euler.points[0]);
        float ctheta = (float) Math.cos(euler.points[1]);
        float stheta = (float) Math.sin(euler.points[1]);
        float cphi = (float) Math.cos(euler.points[2]);
        float sphi = (float) Math.sin(euler.points[2]);

        points3by3[0][0]=cpsi*ctheta;
        points3by3[0][1]=cpsi*stheta*sphi-spsi*cphi;
        points3by3[0][2]=cpsi*stheta*cphi+spsi*sphi;
        points3by3[1][0]=spsi*ctheta;
        points3by3[1][1]=spsi*stheta*sphi+cpsi*cphi;
        points3by3[1][2]=spsi*stheta*cphi-cpsi*sphi;
        points3by3[2][0]=-stheta;
        points3by3[2][1]=ctheta*sphi;
        points3by3[2][2]=ctheta*cphi;

        copyToVector();
    }

    public void toEulerAngles(EulerAngles euler) {
//		currently, only good for theta between -pi/2 and pi/2, and switch to 3/2pi to pi/2
//		rm has to be the rotation matrix from body to earth

        float dotprod= points3by3[2][2]; // dot product of ze and zb
        if (dotprod>=0) {
            euler.points[0] = (float) Math.atan2(points3by3[1][0], points3by3[0][0]);//psi, yaw
            euler.points[1] = (float) Math.asin(-points3by3[2][0]); //theta, pitch
            euler.points[2] = (float) Math.atan2(points3by3[2][1], points3by3[2][2]); //phi, roll
        } else {
            euler.points[0] = (float) Math.atan2(-points3by3[1][0], -points3by3[0][0]);//psi, yaw
            euler.points[1] = (float) Math.PI-(float) Math.asin(-points3by3[2][0]); //theta, pitch
            euler.points[2] = (float) Math.atan2(-points3by3[2][1], -points3by3[2][2]); //phi, roll
        }
    }

    public EulerAngles getEulerAngles() {
//		currently, only good for theta between -pi/2 and pi/2, and switch to 3/2pi to pi/2
//		rm has to be the rotation matrix from body to earth
        EulerAngles euler = new EulerAngles();
        toEulerAngles(euler);
        return euler;
    }

    ///////////////////////////////////////////////////////////
    ////////////////////Axis Angle/////////////////////////////
    ///////////////////////////////////////////////////////////
    public void setFromAxisAngle(AxisAngle n) {
        //n has to be a valid axis angle representation, the first three elements are unit axis representation
        //the last element is theta valid from 0 to pi.
        float ctheta=(float) Math.cos(n.points[3]);
        float stheta=(float) Math.sin(n.points[3]);

        points3by3[0][0]=(1-ctheta)*n.points[0]*n.points[0]+ctheta;
        points3by3[0][1]=(1-ctheta)*n.points[0]*n.points[1]-n.points[2]*stheta;
        points3by3[0][2]=(1-ctheta)*n.points[0]*n.points[2]+n.points[1]*stheta;
        points3by3[1][0]=(1-ctheta)*n.points[1]*n.points[0]+n.points[2]*stheta;
        points3by3[1][1]=(1-ctheta)*n.points[1]*n.points[1]+ctheta;
        points3by3[1][2]=(1-ctheta)*n.points[1]*n.points[2]-n.points[0]*stheta;
        points3by3[2][0]=(1-ctheta)*n.points[2]*n.points[0]-n.points[1]*stheta;
        points3by3[2][1]=(1-ctheta)*n.points[2]*n.points[1]+n.points[0]*stheta;
        points3by3[2][2]=(1-ctheta)*n.points[2]*n.points[2]+ctheta;
        copyToVector();
    }

    public void toAxisAngle(AxisAngle n) {
        //see ref2. 32, NOT TESTED
        n.points[3]=(float) Math.acos(0.5f * (getTrace() - 1));
        if (n.points[3] == 0) {
            //indetermined
            n.points[0]=0;
            n.points[1]=0;
            n.points[2]=0;
        } else if (n.points[3]== (float) Math.PI) {
            //depends on the case which diaganol elements are the largest
            if (points3by3[0][0]>=points3by3[1][1] && points3by3[0][0]>= points3by3[2][2]) {
                n.points[0]=points3by3[0][0]+1;
                n.points[1]=points3by3[0][1];
                n.points[2]=points3by3[0][2];
            } else if (points3by3[1][1]>=points3by3[0][0] && points3by3[1][1]>= points3by3[2][2]) {
                n.points[0]=points3by3[1][0];
                n.points[1]=points3by3[1][1]+1;
                n.points[2]=points3by3[1][2];
            } else {
                n.points[0]=points3by3[2][0];
                n.points[1]=points3by3[2][1];
                n.points[2]=points3by3[2][2]+1;
            }
        } else {
            float n1 = points3by3[2][1]-points3by3[1][2];
            float n2 = points3by3[0][2]-points3by3[2][0];
            float n3 = points3by3[1][0]-points3by3[0][1];

            float mag = (float) Math.sqrt(n1 * n1 + n2 * n2 + n3 * n3);

            n1 /= mag;
            n2 /= mag;
            n3 /= mag;

            n.points[0]=n1;
            n.points[1]=n2;
            n.points[2]=n3;

        }
    }



    public AxisAngle getAxisAngle() {
        //see ref2. 32, NOT TESTED
        AxisAngle n = new AxisAngle();
        toAxisAngle(n);
        return n;
    }

    //////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////


    public void rotateV(Vector3f r1, Vector3f r2) {
        //r2 = M*r1;
        multiplyMV(r1,r2);
    }

    public Vector3f rotateV(Vector3f r1) {
        //r2 = M*r1;
        return multiplyMV(r1);

    }

    ///////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////


    public void toCorrectAxisAngleByGravity(Vector3f accel, AxisAngle axisAngle) {
        //only works if rmin is in standard R b to e form
        //should return axis angle represents R b to e also


        float magG=accel.getNorm();

        if (Math.abs(magG - GravityVector.G)<=GravityVector.TOL_G) {

            Vector3f accelNorm = new Vector3f(accel.points[0], accel.points[1], accel.points[2]);
            Vector3f zeInB = getZe();
            zeInB.normalize();//may not be necessary !!!!!!
            accelNorm.normalize();

            Vector3f correctVector = zeInB.crossProduct(accelNorm); //zeInB x accelnorm

            float dotprod = zeInB.dotProduct(accelNorm);
            float sintheta = correctVector.getNorm();
            float theta = 0.0f;

            if (dotprod >= 0.0f) {
                theta = (float) Math.asin(sintheta);
            } else {
                theta = (float) Math.PI - (float) Math.asin(sintheta);
            }

            theta = -theta; //in order to get R b to e;
            correctVector.normalize();

            axisAngle.points[0]=correctVector.points[0];
            axisAngle.points[1]=correctVector.points[1];
            axisAngle.points[2]=correctVector.points[2];
            axisAngle.points[3]=theta;

        } else {
            axisAngle.setZeroRotation();
        }
    }

    public AxisAngle getCorrectAxisAngleByGravity(Vector3f accel) {
        //only works if rmin is in standard R b to e form
        //should return axis angle represents R b to e also

        AxisAngle axisAngle = new AxisAngle();
        this.toCorrectAxisAngleByGravity(accel, axisAngle);
        return axisAngle;
    }

    //////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////


    public void toDiffRotationMatrixToTarget(RotationMatrix targetRM, RotationMatrix diffRM) {
        //the diff rotation matrix represents from body to earth
        diffRM.set(getTranspose().multiplyMM(targetRM));

    }

    public RotationMatrix getDiffRotationMatrixToTarget(RotationMatrix targetRM) {
        RotationMatrix diffRM = new RotationMatrix();
        toDiffRotationMatrixToTarget(targetRM, diffRM);
        return diffRM;
    }

}
