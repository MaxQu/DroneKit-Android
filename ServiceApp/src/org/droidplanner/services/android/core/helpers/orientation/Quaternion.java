package org.droidplanner.services.android.core.helpers.orientation;

/**
 * Created by mzqu on 8/6/2015.
 */
public class Quaternion extends Vector4f{

    //override the constructor

    public Quaternion() {
        setIdentity();
    }

    public Quaternion(float[] v) {
        super(v);
    }

    public Quaternion(Quaternion q) {
        this.points[0] = q.points[0];
        this.points[1] = q.points[1];
        this.points[2] = q.points[2];
        this.points[3] = q.points[3];
    }

    public Quaternion(float x, float y, float z, float w) {
        super(x,y,z,w);
    }

    public Quaternion(Vector3f v, float w) {
        super(v,w);
    }

    public Quaternion(Vector4f v) {
        super(v);
    }


    //end of constructor
    /////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////
    public void setIdentity() {
        setX(0);
        setY(0);
        setZ(0);
        setW(1);
    }

    public void setZeroRotation() {
        setIdentity();
    }

    public Quaternion clone() {
        Quaternion qclone = new Quaternion();
        qclone.copyVec4(this);
        return qclone;
    }

    /**
     * Copies the values from the given quaternion to this one
     *
     * @param quat The quaternion to copy from
     */
    public void set(Quaternion q) {
        copyVec4(q);
    }
    /////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////
    public Vector3f getZe() {
        //get ze in body coordinate in the corresponding rotation matrix
        Vector3f ze=new Vector3f();
        ze.points[0] = 2 * points[0] * points[2] - 2 * points[1] * points[3];
        ze.points[1] = 2 * points[1] * points[2] + 2 * points[0] * points[3];
        ze.points[2] = 1 - 2 * points[0] * points[0] - 2 * points[1] * points[1];

        return ze;
    }



    ////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////

    public void multiplyQuat(Quaternion q1, Quaternion q3) {
        //see ref.2 page 26
        //    	q2*q1=q3
        if (q3 != q1 && q3 !=this) {
            q3.points[3] = (points[3] * q1.points[3] - points[0] * q1.points[0] - points[1] * q1.points[1] - points[2] * q1.points[2]); //w = w1w2 - x1x2 - y1y2 - z1z2
            q3.points[0] = (points[0] * q1.points[3] + points[3] * q1.points[0] + points[1] * q1.points[2] - points[2] * q1.points[1]); //x = w1x2 + x1w2 + y1z2 - z1y2
            q3.points[1] = (points[1] * q1.points[3] + points[3] * q1.points[1] + points[2] * q1.points[0] - points[0] * q1.points[2]); //y = w1y2 + y1w2 + z1x2 - x1z2
            q3.points[2] = (points[2] * q1.points[3] + points[3] * q1.points[2] + points[0] * q1.points[1] - points[1] * q1.points[0]); //z = w1z2 + z1w2 + x1y2 - y1x2
        } else if (q3 != q1 && q3 == this) {
            float[] t2 = new float[] {points[0],points[1],points[2],points[3]};
            q3.points[3] = (t2[3] * q1.points[3] - t2[0] * q1.points[0] - t2[1] * q1.points[1] - t2[2] * q1.points[2]); //w = w1w2 - x1x2 - y1y2 - z1z2
            q3.points[0] = (t2[0] * q1.points[3] + t2[3] * q1.points[0] + t2[1] * q1.points[2] - t2[2] * q1.points[1]); //x = w1x2 + x1w2 + y1z2 - z1y2
            q3.points[1] = (t2[1] * q1.points[3] + t2[3] * q1.points[1] + t2[2] * q1.points[0] - t2[0] * q1.points[2]); //y = w1y2 + y1w2 + z1x2 - x1z2
            q3.points[2] = (t2[2] * q1.points[3] + t2[3] * q1.points[2] + t2[0] * q1.points[1] - t2[1] * q1.points[0]); //z = w1z2 + z1w2 + x1y2 - y1x2

        } else if (q3 == q1 && q3 !=this) {
            float[] t1 = new float[] {q1.points[0],q1.points[1],q1.points[2],q1.points[3]};
            q3.points[3] = (points[3] * t1[3] - points[0] * t1[0] - points[1] * t1[1] - points[2] * t1[2]); //w = w1w2 - x1x2 - y1y2 - z1z2
            q3.points[0] = (points[0] * t1[3] + points[3] * t1[0] + points[1] * t1[2] - points[2] * t1[1]); //x = w1x2 + x1w2 + y1z2 - z1y2
            q3.points[1] = (points[1] * t1[3] + points[3] * t1[1] + points[2] * t1[0] - points[0] * t1[2]); //y = w1y2 + y1w2 + z1x2 - x1z2
            q3.points[2] = (points[2] * t1[3] + points[3] * t1[2] + points[0] * t1[1] - points[1] * t1[0]); //z = w1z2 + z1w2 + x1y2 - y1x2

        } else {
            float[] t1 = new float[] {q1.points[0],q1.points[1],q1.points[2],q1.points[3]};
            float[] t2 = new float[] {points[0],points[1],points[2],points[3]};
            q3.points[3] = (t2[3] * t1[3] - t2[0] * t1[0] - t2[1] * t1[1] - t2[2] * t1[2]); //w = w1w2 - x1x2 - y1y2 - z1z2
            q3.points[0] = (t2[0] * t1[3] + t2[3] * t1[0] + t2[1] * t1[2] - t2[2] * t1[1]); //x = w1x2 + x1w2 + y1z2 - z1y2
            q3.points[1] = (t2[1] * t1[3] + t2[3] * t1[1] + t2[2] * t1[0] - t2[0] * t1[2]); //y = w1y2 + y1w2 + z1x2 - x1z2
            q3.points[2] = (t2[2] * t1[3] + t2[3] * t1[2] + t2[0] * t1[1] - t2[1] * t1[0]); //z = w1z2 + z1w2 + x1y2 - y1x2
        }
    }

    public Quaternion multiplyQuat(Quaternion q1) {
        Quaternion q3 = new Quaternion();
        q3.points[3] = (points[3] * q1.points[3] - points[0] * q1.points[0] - points[1] * q1.points[1] - points[2] * q1.points[2]); //w = w1w2 - x1x2 - y1y2 - z1z2
        q3.points[0] = (points[0] * q1.points[3] + points[3] * q1.points[0] + points[1] * q1.points[2] - points[2] * q1.points[1]); //x = w1x2 + x1w2 + y1z2 - z1y2
        q3.points[1] = (points[1] * q1.points[3] + points[3] * q1.points[1] + points[2] * q1.points[0] - points[0] * q1.points[2]); //y = w1y2 + y1w2 + z1x2 - x1z2
        q3.points[2] = (points[2] * q1.points[3] + points[3] * q1.points[2] + points[0] * q1.points[1] - points[1] * q1.points[0]); //z = w1z2 + z1w2 + x1y2 - y1x2
        return q3;
    }


    /**
     * Add a quaternion to this quaternion
     *
     * @param input The quaternion that you want to add to this one
     */
    public void addQuat(Quaternion input) {
        addQuat(input, this);
    }


    /**
     * Add this quaternion and another quaternion together and store the result in the output quaternion
     *
     * @param input The quaternion you want added to this quaternion
     * @param output The quaternion you want to store the output in.
     */
    public void addQuat(Quaternion input, Quaternion output) {
        output.setX(getX() + input.getX());
        output.setY(getY() + input.getY());
        output.setZ(getZ() + input.getZ());
        output.setW(getW() + input.getW());
    }




    /**
     * Subtract a quaternion to this quaternion
     *
     * @param input The quaternion that you want to subtracted from this one
     */
    public void subQuat(Quaternion input) {
        subQuat(input, this);
    }

    /**
     * Subtract another quaternion from this quaternion and store the result in the output quaternion
     *
     * @param input The quaternion you want subtracted from this quaternion
     * @param output The quaternion you want to store the output in.
     */
    public void subQuat(Quaternion input, Quaternion output) {
        output.setX(getX() - input.getX());
        output.setY(getY() - input.getY());
        output.setZ(getZ() - input.getZ());
        output.setW(getW() - input.getW());
    }

    public void conjugate() {
        points[0]=-points[0];
        points[1]=-points[1];
        points[2]=-points[2];
        points[3]=points[3];
    }

    public Quaternion getConjugate() {
        Quaternion q= new Quaternion();
        q.points[0]=-points[0];
        q.points[1]=-points[1];
        q.points[2]=-points[2];
        q.points[3]=points[3];
        return q;
    }

    ///////////////////////////////////////////////////////////////////////////////
    ///////////////////////////Rotation Matrix//////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////
    public void setFromRotationMatrix(RotationMatrix rm){
        //see ref2. 35	NOT TESTED
        float q0square=(1.0f/4.0f) *(1+rm.points3by3[0][0]+rm.points3by3[1][1]+rm.points3by3[2][2]);
        if (q0square>0) {
            points[3]=(float) Math.sqrt(q0square);
            points[0]=(1.0f/(4.0f*points[3]))*(rm.points3by3[2][1]-rm.points3by3[1][2]);
            points[1]=(1.0f/(4.0f*points[3]))*(rm.points3by3[0][2]-rm.points3by3[2][0]);
            points[2]=(1.0f/(4.0f*points[3]))*(rm.points3by3[1][0]-rm.points3by3[0][1]);
        } else {
            points[3]=0;
            float q1square=(-1.0f/2.0f)*(rm.points3by3[1][1]+rm.points3by3[2][2]);
            if (q1square>0) {
                points[0]=(float) Math.sqrt(q1square);
                points[1]=rm.points3by3[0][1]/(2*points[0]);
                points[2]=rm.points3by3[0][2]/(2*points[0]);
            } else {
                points[0]=0;
                float q2square =(1.0f/2.0f)*(1-rm.points3by3[2][2]);
                if (q2square > 0) {
                    points[1] = (float) Math.sqrt(q2square);
                    points[2] = rm.points3by3[1][2]/(2*points[1]);
                } else {
                    points[1]=0;
                    points[2]=1;
                }
            }
        }
    }


    public void toRotationMatrix(Matrix3by3 rm) {

        if (rm.length() !=9 || length() !=4) {
            throw new IllegalArgumentException("Array dimension is not right!!");
        }

        float q0;
        float q1 = points[0];
        float q2 = points[1];
        float q3 = points[2];

        if (points.length == 4) {
            q0 = points[3];
        } else {
            q0 = 1 - q1*q1 - q2*q2 - q3*q3;
            q0 = (q0 > 0) ? (float) Math.sqrt(q0) : 0;
        }


        //see ref2. page 35.
        rm.points9by1[0] = 1 - 2 * q2 * q2 - 2 * q3 * q3;
        rm.points9by1[1] = 2 * q1 * q2 - 2 * q3 * q0;
        rm.points9by1[2] = 2 * q1 * q3 + 2 * q2 * q0;

        rm.points9by1[3] = 2 * q1 * q2 + 2 * q3 * q0;
        rm.points9by1[4] = 1 - 2 * q1 * q1 - 2 * q3 * q3;
        rm.points9by1[5] = 2 * q2 * q3 - 2 * q1 * q0;

        rm.points9by1[6] = 2 * q1 * q3 - 2 * q2 * q0;
        rm.points9by1[7] = 2 * q2 * q3 + 2 * q1 * q0;
        rm.points9by1[8] = 1 - 2 * q1 * q1 - 2 * q2 * q2;
        rm.copyToMatrix();
    }

    public RotationMatrix getRotationMatrix() {
        RotationMatrix rm = new RotationMatrix();
        toRotationMatrix(rm);
        return rm;
    }

    ///////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////Axis Angle//////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////
    public void setFromAxisAngle(AxisAngle n) {
        //n has to be a valid axis angle representation, the first three elements are unit axis representation
        points[3]=(float) Math.cos(n.points[3] / 2.0f);
        points[0]=(float) Math.sin(n.points[3] / 2.0f)*n.points[0];
        points[1]=(float) Math.sin(n.points[3] / 2.0f)*n.points[1];
        points[2]=(float) Math.sin(n.points[3] / 2.0f)*n.points[2];
    }

    /**
     * Get an axis angle representation of this quaternion.
     *
     * @param output Vector4f axis angle.
     */
    public void toAxisAngle(Vector4f output) {
        if (getW() > 1) {
            normalize(); // if w>1 acos and sqrt will produce errors, this cant happen if quaternion is normalised
        }
        float angle = 2 * (float) Math.acos(getW());
        float x;
        float y;
        float z;

        float s = (float) Math.sqrt(1 - getW() * getW()); // assuming quaternion normalised then w is less than 1, so term always positive.
        if (s < 0.001) { // test to avoid divide by zero, s is always positive due to sqrt
            // if s close to zero then direction of axis not important
            x = points[0]; // if it is important that axis is normalised then replace with x=1; y=z=0;
            y = points[1];
            z = points[2];
        } else {
            x = points[0] / s; // normalise axis
            y = points[1] / s;
            z = points[2] / s;
        }

        output.points[0] = x;
        output.points[1] = y;
        output.points[2] = z;
        output.points[3] = angle;
    }

    public AxisAngle getAxisAngle() {
        AxisAngle output = new AxisAngle();
        toAxisAngle(output);
        return output;
    }

    /////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////Euler Angles/////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////

    public void setFromEulerAngles(EulerAngles euler){

        float cpsi = (float) Math.cos(euler.points[0] / 2.0f);
        float spsi = (float) Math.sin(euler.points[0] / 2.0f);
        float ctheta = (float) Math.cos(euler.points[1] / 2.0f);
        float stheta = (float) Math.sin(euler.points[1] / 2.0f);
        float cphi = (float) Math.cos(euler.points[2] / 2.0f);
        float sphi = (float) Math.sin(euler.points[2] / 2.0f);

        points[3]=cpsi*ctheta*cphi+spsi*stheta*sphi;
        points[0]=cpsi*ctheta*sphi-spsi*stheta*cphi;
        points[1]=cpsi*stheta*cphi+spsi*ctheta*sphi;
        points[2]=spsi*ctheta*cphi-cpsi*stheta*sphi;
    }


    public void toEulerAngles(Vector3f euler){
        //see ref2. 39, for body zyx procedure
        //only valid for theta (pitch) to be within (-pi/2, pi/2)
        float dotprod = 1-2*(points[0]*points[0]+points[1]*points[1]); //dot prod of ze and zb
        if (dotprod>=0) {
            euler.points[0] = (float) Math.atan2(points[0] * points[1] + points[3] * points[2], 0.5f - (points[1] * points[1] + points[2] * points[2]));
            euler.points[1] = (float) Math.asin(-2 * (points[0] * points[2] - points[3] * points[1]));
            euler.points[2] = (float) Math.atan2(points[1] * points[2] + points[3] * points[0], 0.5f - (points[0] * points[0] + points[1] * points[1]));
        } else {
            euler.points[0] = (float) Math.atan2(-(points[0] * points[1] + points[3] * points[2]), -0.5f + (points[1] * points[1] + points[2] * points[2]));
            euler.points[1] = (float) Math.PI - (float) Math.asin(-2 * (points[0] * points[2] - points[3] * points[1]));
            euler.points[2] = (float) Math.atan2(-(points[1] * points[2] + points[3] * points[0]), -0.5f + (points[0] * points[0] + points[1] * points[1]));
        }
    }

    public EulerAngles getEulerAngles(){
        //see ref2. 39, for body zyx procedure
        //only valid for theta (pitch) to be within (-pi/2, pi/2)
        EulerAngles euler = new EulerAngles();
        toEulerAngles(euler);
        return euler;
    }



    ///////////////////////////////////////////////
    //////////////////////////////////////////////
    /////////////////////////////////////////////







    public void rotateV(Vector3f r1, Vector3f r2) {
        //r2q=q * r1q * conj(q);
        Quaternion r1q = new Quaternion(r1.points[0],r1.points[1],r1.points[2],0);
        Quaternion r2q = this.multiplyQuat(r1q.multiplyQuat(this.getConjugate()));


        r2.points[0]=r2q.points[0];
        r2.points[1]=r2q.points[1];
        r2.points[2]=r2q.points[2];
    }

    public Vector3f rotateV(Vector3f r1) {
        //r2q=q * r1q * conj(q);
        Vector3f r2 = new Vector3f();
        rotateV(r1,r2);
        return r2;
    }

    public float[] toArray() {
        return points;
    }


    public Quaternion getSlerp(Quaternion q2) {
        //TODO
        return q2;
    }

    ///////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////

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


    ////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////

    public void toDiffQuaternionToTarget(Quaternion targetQ,Quaternion diffQ) {
        //the diffQ represents rotation from body to earth
        diffQ.set(getConjugate().multiplyQuat(targetQ));
    }

    public Quaternion getDiffQuaternionToTarget(Quaternion targetQ) {
        //the diffQ represents rotation from body to earth
        Quaternion diffQ = new Quaternion();
        toDiffQuaternionToTarget(targetQ,diffQ);
        return diffQ;
    }






}