package artisynth.core.femmodels;

import maspack.matrix.DenseMatrixBase;
import maspack.matrix.Matrix3x1Block;
import maspack.matrix.Matrix6d;
import maspack.matrix.MatrixBlock;
import maspack.matrix.MatrixNd;
import maspack.matrix.VectorBase;

/**
 * Base class for storing information that's specific between a node and its 
 * neighbor (e.g. stiffness)
 */
public abstract class NodeNeighbor {

   protected int myBlkNum;
   protected int myRefCnt;
   
   // block used in the incompressibility constraint matrix.
   public Matrix3x1Block myDivBlk;
   // Matrix3x1Block myDivBlk1;
   
   public abstract void zeroStiffness();
   
   public abstract void addStiffness (DenseMatrixBase K);

   public abstract DenseMatrixBase getK();
   
   public abstract MatrixBlock getDivBlk();
   
   public abstract void setDivBlk(MatrixBlock blk);
   
   public abstract void setTransposedStiffness(NodeNeighbor nbr);

   public void setRefCount(int count) {
      myRefCnt = count;
   }
   
   public void increaseRefCount() {
      myRefCnt++;
   }
   
   public void decreaseRefCount() {
      myRefCnt--;
   }
   
   public int getBlockNumber() {
      return myBlkNum;
   }
   
   public void setBlockNumber (int num) {
      myBlkNum = num;
   }
   
   public abstract FemNode3d getNode();
   
   public abstract void addVelJacobian (DenseMatrixBase blk, double s, 
      double stiffnessDamping, double massDamping);

   public abstract void addPosJacobian (DenseMatrixBase blk, double s);
   
   public abstract void addDampingForce (VectorBase fd);

   public abstract void addDilationalStiffness (
      double kp, VectorBase intGi, VectorBase intGj);
   
   public abstract void addDilationalStiffness (
      double[] Kp, MatrixBlock GT_i, MatrixBlock GT_j);
   
   public abstract void addDilationalStiffness (
      MatrixNd Rinv, MatrixBlock GT_i, MatrixBlock GT_j);
   
   public abstract void addIncompressibilityStiffness (
      double s, VectorBase intGi, VectorBase intGj);

   //public abstract void addMaterialStiffness;
   
   public abstract void addPressureStiffness( VectorBase gi, double p,
      VectorBase gj, double dv);
}
