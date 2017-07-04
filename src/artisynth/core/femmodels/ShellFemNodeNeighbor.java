package artisynth.core.femmodels;

import maspack.matrix.DenseMatrixBase;
import maspack.matrix.Matrix3d;
import maspack.matrix.Matrix6d;
import maspack.matrix.MatrixBlock;
import maspack.matrix.MatrixNd;
import maspack.matrix.SymmetricMatrix3d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorBase;
import maspack.matrix.VectorNd;

public class ShellFemNodeNeighbor extends NodeNeighbor {
   protected ShellFemNode3d myNode;
   protected Matrix6d myK;
   
   // Extra K matrix components for which we don't want to apply 
   // stiffness damping.
   protected Matrix6d myKX; 
 
   
   
   
   public ShellFemNodeNeighbor(ShellFemNode3d node) {
      myNode = node;
      myK = new Matrix6d();
      myRefCnt = 1;
   }
   
   @Override
   public void zeroStiffness() {
      myK.setZero ();
      if (myKX != null) {
         myKX.setZero();
      }
   }
   
   @Override
   public void addStiffness(DenseMatrixBase K) {
      myK.add((Matrix6d)K);
   }
   
   @Override
   public Matrix6d getK() {
      return myK;
   }
   
   @Override
   public MatrixBlock getDivBlk () {
      throw new RuntimeException("Unimplemented");
   }

   @Override
   public void setDivBlk (MatrixBlock blk) {
      throw new RuntimeException("Unimplemented");
   }
   
   /**
    * Set the stiffness of this node neighbour to the transpose of 
    * the stiffness component of another node neighbour. Transpose is done
    * to abide to the symmetrical global stiffness matrix. 
    * @param nbr
    */
   @Override
   public void setTransposedStiffness(NodeNeighbor nbr) {
      ShellFemNodeNeighbor sNbr = (ShellFemNodeNeighbor) nbr;
      
      myK.transpose( sNbr.myK );
      if (sNbr.myKX != null) {
         if (myKX == null) {
            myKX = new Matrix6d();
         }
         myKX.transpose(sNbr.myKX);
      }
   }
   
   @Override
   public ShellFemNode3d getNode() {
      return myNode;
   }
   
   /**
    * Apply both stiffness damping and diagonal mass damping from this
    * node neighbour to a given 6x6 block.
    * 
    * @param blk
    * @param s
    * @param stiffnessDamping
    * @param massDamping
    */
   @Override
   public void addVelJacobian (
      DenseMatrixBase blk, double s, double stiffnessDamping,
      double massDamping) {
      Matrix6d blk6 = (Matrix6d) blk;
      
      blk6.scaledAdd(-s * stiffnessDamping, myK);
      if (massDamping != 0) {
         double d = -s * massDamping * myNode.getMass();
         blk6.m00 += d;
         blk6.m11 += d;
         blk6.m22 += d;
         blk6.m33 += d;
         blk6.m44 += d;
         blk6.m55 += d;
      }
   }
   
   /**
    * Apply stiffness from this node neighour to a given 6x6 block. 
    * 
    * @param blk
    * @param s
    */
   @Override
   public void addPosJacobian (DenseMatrixBase blk, double s) {
      Matrix6d blk6 = (Matrix6d) blk;
      blk6.scaledAdd (-s, myK);
      if (myKX != null) {
         blk6.scaledAdd(-s, myKX);
      }
   }
   
   /**
    * Add the damping force (node neighbour stiffness * velocity) to 
    * a given force vector.
    * 
    * @param fd
    */
   @Override
   public void addDampingForce(VectorBase fd) {
      VectorNd fd6 = (VectorNd) fd;
      
      // Get node velocity.
      double[] vBuf = new double[6];
      myNode.getVelState(vBuf, 0);
      VectorNd v = new VectorNd(vBuf);
      
      // stiffness * velocity
      VectorNd kv = new VectorNd();
      kv.mul(myK, v);
      
      fd6.add(kv);
   }
   
   @Override
   public void addDilationalStiffness (
      double kp, VectorBase intGi, VectorBase intGj) {
      throw new RuntimeException("Unimplemented");
   }
   
   @Override
   public void addDilationalStiffness (
      double[] Kp, MatrixBlock GT_i, MatrixBlock GT_j) {
      throw new RuntimeException("Unimplemented");
   }
   
   @Override
   public void addDilationalStiffness (
      MatrixNd Rinv, MatrixBlock GT_i, MatrixBlock GT_j) {
      throw new RuntimeException("Unimplemented");
   }
   
   @Override
   public void addIncompressibilityStiffness (
      double s, VectorBase intGi, VectorBase intGj) {
      throw new RuntimeException("Unimplemented");
   }

   @Override
   public void addPressureStiffness( VectorBase gi, double p,
      VectorBase gj, double dv) {
      if (FemModel3d.noIncompressStiffnessDamping) {
         if (myKX == null) {
            myKX = new Matrix6d();
         }
         throw new RuntimeException("Unimplemented");
         //FemUtilities.addPressureStiffness (myKX, gi, p, gj, dv);  
         //FemUtilities.addPressureStiffness (myK, gi, -p, gj, dv);  
      }
   }
   
   /**
    * Add weighted material stiffness for this i,j node neighbor pair, 
    * relative to a particular integration point of the shell element.
    * 
    * This material stiffness also accounts for geometrical stiffness.
    * 
    * Standard pressure stiffness is added as well (non-shell specific).
    * 
    * @param iN
    * Shape function of i-th node and integration point.
    * 
    * @param jN
    * Shape function of j-th node and integration point.
    * 
    * @param idN
    * Derivative of shape function of i-th node and integration point.
    * 
    * @param jdN 
    * Derivative of shape function of j-th node and integration point.
    * 
    * @param dv
    * Weighted determinant of integration point jacobian.
    * 
    * @param t
    * t-component of (r,s,t) integration point coordinates (i.e. gauss point)
    * 
    * @param gct 
    * Contravariant base vectors of integration point.
    * 
    * @param matStress
    * Material stress of integration point.
    * 
    * @param matTangent
    * Material tangent of integration point.
    * 
    * @param gi 
    * Shape function gradient of i-th node and integration point.
    * 
    * @param gj 
    * Shape function gradient of j-th node and integration point.
    * 
    * @param p
    * Pressure.
    * 
    * @postcond
    * this.myK (6x6 stiffness block for this i-j node pair) is increased.
    */
   public void addMaterialStiffness(
      double iN, double jN, Vector3d idN, Vector3d jdN, double dv, double t,
      Vector3d[] gct, SymmetricMatrix3d matStress, Matrix6d matTangent, 
      Vector3d gi, Vector3d gj, double p) {
//      FemUtilities.addShellMaterialStiffness(
//         myK, iN, jN, idN, jdN, dv, t, gct, matStress, matTangent);
      
      addPressureStiffness(gi, p, gj, dv);
   }


}
