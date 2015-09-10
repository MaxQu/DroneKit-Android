package org.droidplanner.services.android.core.helpers.orientation;

/**
 * Created by mzqu on 8/6/2015.
 */
import java.util.Arrays;

public class Matrix3by3 {
    //all initialized as identity matrix
    protected float[][] points3by3= new float[][] {{1,0,0},{0,1,0},{0,0,1}};
    protected float[] points9by1 = new float[] {1,0,0,0,1,0,0,0,1};



    public Matrix3by3(float[] M) {
        System.arraycopy(M, 0, points9by1, 0, 9);
        copyToMatrix();

    }

    public Matrix3by3(float[][] M) {
        points3by3[0][0] = M[0][0];
        points3by3[0][1] = M[0][1];
        points3by3[0][2] = M[0][2];
        points3by3[1][0] = M[1][0];
        points3by3[1][1] = M[1][1];
        points3by3[1][2] = M[1][2];
        points3by3[2][0] = M[2][0];
        points3by3[2][1] = M[2][1];
        points3by3[2][2] = M[2][2];
        copyToVector();

    }

    public Matrix3by3(Matrix3by3 M) {
        points3by3[0][0] = M.points3by3[0][0];
        points3by3[0][1] = M.points3by3[0][1];
        points3by3[0][2] = M.points3by3[0][2];
        points3by3[1][0] = M.points3by3[1][0];
        points3by3[1][1] = M.points3by3[1][1];
        points3by3[1][2] = M.points3by3[1][2];
        points3by3[2][0] = M.points3by3[2][0];
        points3by3[2][1] = M.points3by3[2][1];
        points3by3[2][2] = M.points3by3[2][2];
        copyToVector();
    }

    public Matrix3by3(float[] row1, float[] row2, float[] row3) {
//		int rows = numRows();
        int cols = numCols();

        for (int j=0; j<cols; j++){
            points3by3[0][j]=row1[j];
        }

        for (int j=0; j<cols; j++){
            points3by3[1][j]=row2[j];
        }

        for (int j=0; j<cols; j++){
            points3by3[2][j]=row3[j];
        }

        copyToVector();

    }

    public Matrix3by3(Vector3f row1, Vector3f row2, Vector3f row3) {
//		int rows = numRows();
        int cols = numCols();

        for (int j=0; j<cols; j++){
            points3by3[0][j]=row1.points[j];
        }

        for (int j=0; j<cols; j++){
            points3by3[1][j]=row2.points[j];
        }

        for (int j=0; j<cols; j++){
            points3by3[2][j]=row3.points[j];
        }

        copyToVector();

    }

    public Matrix3by3() {
        points3by3[0][0] = 0;
        points3by3[0][1] = 0;
        points3by3[0][2] = 0;
        points3by3[1][0] = 0;
        points3by3[1][1] = 0;
        points3by3[1][2] = 0;
        points3by3[2][0] = 0;
        points3by3[2][1] = 0;
        points3by3[2][2] = 0;
        copyToVector();
    }

    //end of constructor

    public void copyToVector() {
        points9by1[0]=points3by3[0][0];
        points9by1[1]=points3by3[0][1];
        points9by1[2]=points3by3[0][2];
        points9by1[3]=points3by3[1][0];
        points9by1[4]=points3by3[1][1];
        points9by1[5]=points3by3[1][2];
        points9by1[6]=points3by3[2][0];
        points9by1[7]=points3by3[2][1];
        points9by1[8]=points3by3[2][2];
    }


    public void copyToMatrix() {
        points3by3[0][0] =points9by1[0];
        points3by3[0][1] =points9by1[1];
        points3by3[0][2] =points9by1[2];
        points3by3[1][0] =points9by1[3];
        points3by3[1][1] =points9by1[4];
        points3by3[1][2] =points9by1[5];
        points3by3[2][0] =points9by1[6];
        points3by3[2][1] =points9by1[7];
        points3by3[2][2] =points9by1[8];

    }

    public void clone(Matrix3by3 M){
        points3by3[0][0] = M.points3by3[0][0];
        points3by3[0][1] = M.points3by3[0][1];
        points3by3[0][2] = M.points3by3[0][2];
        points3by3[1][0] = M.points3by3[1][0];
        points3by3[1][1] = M.points3by3[1][1];
        points3by3[1][2] = M.points3by3[1][2];
        points3by3[2][0] = M.points3by3[2][0];
        points3by3[2][1] = M.points3by3[2][1];
        points3by3[2][2] = M.points3by3[2][2];
        copyToVector();
    }

    public void clone(float[] M) {
        System.arraycopy(M, 0, points9by1, 0, 9);
        copyToMatrix();
    }

    public void clone(float[][] M) {
        points3by3[0][0] = M[0][0];
        points3by3[0][1] = M[0][1];
        points3by3[0][2] = M[0][2];
        points3by3[1][0] = M[1][0];
        points3by3[1][1] = M[1][1];
        points3by3[1][2] = M[1][2];
        points3by3[2][0] = M[2][0];
        points3by3[2][1] = M[2][1];
        points3by3[2][2] = M[2][2];
        copyToVector();
    }


    public int numRows() {
        return points3by3.length;
    }

    public int numCols() {
        return points3by3[0].length;
    }

    public int length() {
        return points9by1.length;
    }

    public void reset() {
        for (int i=0; i<numRows(); i++){
            for (int j=0;j<numCols();j++){
                points3by3[i][j]=0.00f;
            }
        }
        copyToVector();
    }

    public void makeMfromRowV(float[]... vectors) {
        int num_vectors=vectors.length;
        int length_vector=vectors[0].length;
        int rows = numRows();
        int cols = numCols();

        if (rows!=num_vectors || cols!=length_vector) {
            throw new IllegalArgumentException("Cannot evenly distribute elements into the new arrays!!");
        }

        for (int i=0; i<rows; i++){
            for (int j=0; j<cols; j++) {
                points3by3[i][j]=vectors[i][j];
            }
        }
        copyToVector();
    }


    public void makeMfromRowV(Vector3f... vectors) {
        int num_vectors=vectors.length;
        int length_vector=vectors[0].length();
        int rows = numRows();
        int cols = numCols();

        if (rows!=num_vectors || cols!=length_vector) {
            throw new IllegalArgumentException("Cannot evenly distribute elements into the new arrays!!");
        }

        for (int i=0; i<rows; i++){
            for (int j=0; j<cols; j++) {
                points3by3[i][j]=vectors[i].points[j];
            }
        }
        copyToVector();
    }

    public void multiplyMM(Matrix3by3 M2, Matrix3by3 M3) {
        //always put the output at the end
        int aRows = numRows();
        int aCols = numCols();
        int bRows = M2.numRows();
        int bCols= M2.numCols();

        if (aCols != bRows) {
            throw new IllegalArgumentException("A:Rows: " + aCols + " did not match B:Columns " + bRows + ".");
        }

        if (M3!=M2 && M3!=this) {

            M3.reset();

            for (int i = 0; i < aRows; i++) { // aRow
                for (int j=0; j<bCols;j++){
                    for (int k = 0; k < aCols; k++) { // aColumn
                        M3.points3by3[i][j] += points3by3[i][k] * M2.points3by3[k][j];
                    }
                }

            }
        } else if (M3!=M2 && M3==this) {
            Matrix3by3 M1temp = new Matrix3by3(this);

            M3.reset();

            for (int i = 0; i < aRows; i++) { // aRow
                for (int j=0; j < bCols; j++){
                    for (int k = 0; k < aCols; k++) { // aColumn
                        M3.points3by3[i][j] += M1temp.points3by3[i][k] * M2.points3by3[k][j];
                    }
                }

            }

        } else if(M3==M2 && M3!=this) {
            Matrix3by3 M2temp = new Matrix3by3(M2);

            M3.reset();

            for (int i = 0; i < aRows; i++) { // aRow
                for (int j=0; j < bCols; j++){
                    for (int k = 0; k < aCols; k++) { // aColumn
                        M3.points3by3[i][j] += points3by3[i][k] * M2temp.points3by3[k][j];
                    }
                }

            }

        } else {

            Matrix3by3 M1temp = new Matrix3by3(this);
            Matrix3by3 M2temp = new Matrix3by3(M2);

            M3.reset();

            for (int i = 0; i < aRows; i++) { // aRow
                for (int j=0; j < bCols; j++){
                    for (int k = 0; k < aCols; k++) { // aColumn
                        M3.points3by3[i][j] += M1temp.points3by3[i][k] * M2temp.points3by3[k][j];
                    }
                }

            }
        }
        M3.copyToVector();
    }


    public Matrix3by3 multiplyMM(Matrix3by3 M2) {
        //always put the output at the end
        Matrix3by3 M3 = new Matrix3by3();
        multiplyMM(M2,M3);
        M3.copyToVector();
        return M3;

    }


    public void multiplyMV(Vector3f v1, Vector3f v2){

        //r2 = M *r1
        int aRows = numRows();
        int aCols = numCols();
        int bRows = v1.length();
        if (aCols != bRows) {
            throw new IllegalArgumentException("A:Rows: " + aCols + " did not match B:Columns " + bRows + ".");
        }

        if (v1!=v2) {

            v2.reset();

            for (int i = 0; i < aRows; i++) { // aRow
                for (int k = 0; k < aCols; k++) { // aColumn
                    v2.points[i] += points3by3[i][k] * v1.points[k];
                }
            }
        } else {
            Vector3f v1temp = new Vector3f(v1);


            v2.reset();

            for (int i = 0; i < aRows; i++) { // aRow
                for (int k = 0; k < aCols; k++) { // aColumn
                    v2.points[i] += points3by3[i][k] * v1temp.points[k];
                }
            }
        }
    }

    public Vector3f multiplyMV(Vector3f v1) {
        Vector3f v2 = new Vector3f();
        multiplyMV(v1,v2);
        return v2;
    }


    public float[] toArray9by1() {
        return this.points9by1;
    }


    public float[][] toArray3by3() {
        return this.points3by3;
    }


    public String toString() {
        return Arrays.deepToString(points3by3);
    }


    public void transpose() {
        int rows=numRows();
        int cols=numCols();
        Matrix3by3 temp = new Matrix3by3(this);
        for (int i=0; i<rows; i++){
            for (int j=0;j<cols;j++){
                points3by3[i][j]=temp.points3by3[j][i];
            }
        }
        copyToVector();

    }

    public Matrix3by3 getTranspose() {
        Matrix3by3 Mout = new Matrix3by3();
        Mout.clone(this);
        Mout.transpose();
        Mout.copyToMatrix();
        return Mout;
    }

    public float getTrace() {
        return points3by3[0][0]+points3by3[1][1]+points3by3[2][2];
    }

}
