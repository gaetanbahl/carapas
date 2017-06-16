/**
 * Copyright (c) 2014, by the Authors: John E Lloyd (UBC)
 *
 * This software is freely available under a 2-clause BSD license. Please see
 * the LICENSE file in the ArtiSynth distribution directory for details.
 */
package artisynth.core.femmodels;

import maspack.matrix.Matrix3d;
import maspack.matrix.Matrix3x1Block;
import maspack.matrix.Matrix6d;
import maspack.matrix.MatrixBlock;
import maspack.matrix.MatrixNd;
import maspack.matrix.SymmetricMatrix3d;
import maspack.matrix.Vector3d;

public class FemNodeNeighbor {
   protected FemNode3d myNode;
   protected Matrix3d myK;
   // Extra K matrix components for which we don't want to apply stiffness damping
   protected Matrix3d myKX; 
   
   //protected Matrix3x3Block myBlk;
   protected int myBlkNum;
   protected int myRefCnt;
   // block used in the incompressibility constraint matrix.
   protected Matrix3x1Block myDivBlk;
   // Matrix3x1Block myDivBlk1;

   public void zeroStiffness() {
      myK.setZero();
      if (myKX != null) {
         myKX.setZero();
      }
   }
   
   public void addStiffness (Matrix3d K) {
      myK.add (K);
   }

//   public void getStiffness (Matrix3d K){
//      K.set (myK);
//      if (myKX != null) {
//         K.add (myKX);
//      }
//   }
   
   public Matrix3d getK()  {
      return myK;
   }
   
   public Matrix3x1Block getDivBlk() {
      return myDivBlk;
   }
   
   public void setDivBlk(Matrix3x1Block blk) {
      myDivBlk = blk;
   }

//    public void addNondampedStiffness (Matrix3d K) {
//       myKX.add (K);
//    }

   /** 
    * Sets the stiffness components of this node neighbour to the transpose of
    * the stiffness components of another node neighbour. 
    */
   public void setTransposedStiffness (FemNodeNeighbor nbr) {
      myK.transpose (nbr.myK);
      if (nbr.myKX != null) {
         if (myKX == null) {
            myKX = new Matrix3d();
         }
         myKX.transpose (nbr.myKX);
      }
   }
   
   public void setRefCount(int count) {
      myRefCnt = count;
   }
   
   public void increaseRefCount() {
      myRefCnt++;
   }
   
   public void decreaseRefCount() {
      myRefCnt--;
   }
   

   public FemNodeNeighbor (FemNode3d node) {
      myNode = node;
      myK = new Matrix3d();
      myRefCnt = 1;
   }

//   public void setBlock (Matrix3x3Block blk) {
//      myBlk = blk;
//   }
   
   public void setBlockNumber (int num) {
      myBlkNum = num;
   }

   public int getBlockNumber() {
      return myBlkNum;
   }
   
//   public void addVelJacobian (
//      double s, double stiffnessDamping, double massDamping) {
//      addVelJacobian (myBlk, s, stiffnessDamping, massDamping);
//   }

   public void addVelJacobian (
      Matrix3d blk, double s, double stiffnessDamping, double massDamping) {
      // System.out.println (
      // "addVelJacobian: myK=\n" + myK.toString("%10.5f"));
      blk.scaledAdd (-s * stiffnessDamping, myK, blk);
      //blk.scaledAdd (-s * stiffnessDamping, myKX, blk);
      if (massDamping != 0) {
//         if (blk == null) {
//            System.out.println ("null block");
//         }
         //double d = -s * massDamping * myNode.getEffectiveMass();
         double d = -s * massDamping * myNode.getMass();
         blk.m00 += d;
         blk.m11 += d;
         blk.m22 += d;
      }
   }

   public FemNode3d getNode() {
      return myNode;
   }

//   public void addPosJacobian (SparseNumberedBlockMatrix S, double s) {
//      addPosJacobian ((Matrix3x3Block)S.getBlockByNumber(myBlkNum), s);
//   }

   public void addPosJacobian (Matrix3d blk, double s) {
      blk.scaledAdd (-s, myK, blk);
      if (myKX != null) {
         blk.scaledAdd (-s, myKX, blk);
      }
   }

   public void addDampingForce (Vector3d fd) {
      fd.mulAdd (myK, myNode.getVelocity(), fd);
   }

   public void addDilationalStiffness (
      double kp, Vector3d intGi, Vector3d intGj) {

      if (FemModel3d.noIncompressStiffnessDamping) {
         if (myKX == null) {
            myKX = new Matrix3d();
         }
         FemUtilities.addDilationalStiffness (myKX, kp, intGi, intGj);
      }
      else {
         FemUtilities.addDilationalStiffness (myK, kp, intGi, intGj);
      }
      
   }
   
   public void addDilationalStiffness (
      double[] Kp, MatrixBlock GT_i, MatrixBlock GT_j) {

      if (FemModel3d.noIncompressStiffnessDamping) {
         if (myKX == null) {
            myKX = new Matrix3d();
         }
         FemUtilities.addDilationalStiffness (myKX, Kp, GT_i, GT_j);
      }
      else {
         FemUtilities.addDilationalStiffness (myK, Kp, GT_i, GT_j);
      }
      
   }
   
   public void addDilationalStiffness (
      MatrixNd Rinv, MatrixBlock GT_i, MatrixBlock GT_j) {

      if (FemModel3d.noIncompressStiffnessDamping) {
         if (myKX == null) {
            myKX = new Matrix3d();
         }
         FemUtilities.addDilationalStiffness (myKX, Rinv, GT_i, GT_j);
      }
      else {
         FemUtilities.addDilationalStiffness (myK, Rinv, GT_i, GT_j);
      }
      
   }
   
   public void addIncompressibilityStiffness (
      double s, Vector3d intGi, Vector3d intGj) {

      if (FemModel3d.noIncompressStiffnessDamping) {
         if (myKX == null) {
            myKX = new Matrix3d();
         }
         FemUtilities.addIncompressibilityStiffness (myKX, s, intGi, intGj);
      }
      else {
         FemUtilities.addIncompressibilityStiffness (myK, s, intGi, intGj);
      }
      
   }

   public void addMaterialStiffness (
      Vector3d gi, Matrix6d D, double p,
      SymmetricMatrix3d sig, Vector3d gj, double dv) {

      FemUtilities.addMaterialStiffness (myK, gi, D, sig, gj, dv);   
      addPressureStiffness(gi, p, gj, dv);
   }
   
   // Sanchez, April 27, 2013
   // Separated pressure term so I can compute incompressibility component separately
   public void addMaterialStiffness(Vector3d gi, Matrix6d D,
      SymmetricMatrix3d sig, Vector3d gj, double dv) {
      FemUtilities.addMaterialStiffness (myK, gi, D, sig, gj, dv);
   }
   
   public void addMaterialStiffness(Vector3d gi, Matrix6d D, Vector3d gj, double dv) {
      FemUtilities.addMaterialStiffness (myK, gi, D, gj, dv);
   }

   /**
    * Geometric strain-based stiffess
    */
   public void addGeometricStiffness (
      Vector3d gi, SymmetricMatrix3d sig, Vector3d gj, double dv) {
      FemUtilities.addGeometricStiffness (myK, gi, sig, gj, dv);   
   }
   
   public void addPressureStiffness( Vector3d gi, double p,
      Vector3d gj, double dv) {
      
      if (FemModel3d.noIncompressStiffnessDamping) {
         if (myKX == null) {
            myKX = new Matrix3d();
         }
         FemUtilities.addPressureStiffness (myKX, gi, p, gj, dv);  
         FemUtilities.addPressureStiffness (myK, gi, -p, gj, dv);  
      }
      
   }
   
   /**
    * Add weighted material stiffness for this i,j node neighbor pair, 
    * relative to a particular integration point of the shell element.
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
    * this.myK (3x3 stiffness block for this i-j node pair) is increased.
    */
   public void addShellMaterialStiffness(
      double iN, double jN, Vector3d idN, Vector3d jdN, double dv, double t,
      Vector3d[] gct, SymmetricMatrix3d matStress, Matrix6d matTangent, 
      Vector3d gi, Vector3d gj, double p) {
      FemUtilities.addShellMaterialStiffness(
         myK, iN, jN, idN, jdN, dv, t, gct, matStress, matTangent);
      
      addPressureStiffness(gi, p, gj, dv);
   }
}
