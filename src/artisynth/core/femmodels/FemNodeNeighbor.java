/**
 * Copyright (c) 2014, by the Authors: John E Lloyd (UBC)
 *
 * This software is freely available under a 2-clause BSD license. Please see
 * the LICENSE file in the ArtiSynth distribution directory for details.
 */
package artisynth.core.femmodels;

import maspack.matrix.DenseMatrixBase;
import maspack.matrix.Matrix3d;
import maspack.matrix.Matrix3x1Block;
import maspack.matrix.Matrix6d;
import maspack.matrix.MatrixBlock;
import maspack.matrix.MatrixNd;
import maspack.matrix.SymmetricMatrix3d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorBase;

public class FemNodeNeighbor extends NodeNeighbor {
   protected FemNode3d myNode;
   protected Matrix3d myK;
   
   // Extra K matrix components for which we don't want to apply 
   // stiffness damping.
   protected Matrix3d myKX; 
   
   //protected Matrix3x3Block myBlk;

   
   
   public FemNodeNeighbor (FemNode3d node) {
      myNode = node;
      myK = new Matrix3d();
      myRefCnt = 1;
   }
   
   @Override
   public void zeroStiffness() {
      myK.setZero();
      if (myKX != null) {
         myKX.setZero();
      }
   }
   
   @Override
   public void addStiffness (DenseMatrixBase K) {
      myK.add ((Matrix3d)K);
   }

//   public void getStiffness (Matrix3d K){
//      K.set (myK);
//      if (myKX != null) {
//         K.add (myKX);
//      }
//   }
   
   @Override
   public Matrix3d getK()  {
      return myK;
   }
   
   @Override
   public Matrix3x1Block getDivBlk() {
      return myDivBlk;
   }
   
   @Override
   public void setDivBlk(MatrixBlock blk) {
      myDivBlk = (Matrix3x1Block)blk;
   }

//    public void addNondampedStiffness (Matrix3d K) {
//       myKX.add (K);
//    }

   /** 
    * Sets the stiffness components of this node neighbour to the transpose of
    * the stiffness components of another node neighbour. 
    */
   public void setTransposedStiffness (NodeNeighbor nbr) {
      FemNodeNeighbor fNbr = (FemNodeNeighbor) nbr;
      
      myK.transpose (fNbr.myK);
      if (fNbr.myKX != null) {
         if (myKX == null) {
            myKX = new Matrix3d();
         }
         myKX.transpose (fNbr.myKX);
      }
   }

//   public void setBlock (Matrix3x3Block blk) {
//      myBlk = blk;
//   }
   
//   public void addVelJacobian (
//      double s, double stiffnessDamping, double massDamping) {
//      addVelJacobian (myBlk, s, stiffnessDamping, massDamping);
//   }

   @Override
   public void addVelJacobian (
      DenseMatrixBase blk, double s, double stiffnessDamping,
      double massDamping) {
      Matrix3d blk3 = (Matrix3d) blk;
      // System.out.println (
      // "addVelJacobian: myK=\n" + myK.toString("%10.5f"));
      blk3.scaledAdd (-s * stiffnessDamping, myK, blk3);
      //blk3.scaledAdd (-s * stiffnessDamping, myKX, blk3);
      if (massDamping != 0) {
//         if (blk3 == null) {
//            System.out.println ("null block");
//         }
         //double d = -s * massDamping * myNode.getEffectiveMass();
         double d = -s * massDamping * myNode.getMass();
         blk3.m00 += d;
         blk3.m11 += d;
         blk3.m22 += d;
      }
   }

   @Override
   public FemNode3d getNode() {
      return myNode;
   }

//   public void addPosJacobian (SparseNumberedBlockMatrix S, double s) {
//      addPosJacobian ((Matrix3x3Block)S.getBlockByNumber(myBlkNum), s);
//   }

   @Override
   public void addPosJacobian (DenseMatrixBase blk, double s) {
      Matrix3d blk3 = (Matrix3d) blk;
      blk3.scaledAdd (-s, myK, blk3);
      if (myKX != null) {
         blk3.scaledAdd (-s, myKX, blk3);
      }
   }

   @Override
   public void addDampingForce (VectorBase fd) {
      Vector3d fd3 = (Vector3d) fd;
      fd3.mulAdd (myK, myNode.getVelocity(), fd3);
   }

   @Override
   public void addDilationalStiffness (
      double kp, VectorBase intGi, VectorBase intGj) {

      Vector3d intGi3 = (Vector3d) intGi; 
      Vector3d intGj3 = (Vector3d) intGj;
      
      if (FemModel3d.noIncompressStiffnessDamping) {
         if (myKX == null) {
            myKX = new Matrix3d();
         }
         FemUtilities.addDilationalStiffness (myKX, kp, intGi3, intGj3);
      }
      else {
         FemUtilities.addDilationalStiffness (myK, kp, intGi3, intGj3);
      }
      
   }
   
   @Override
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
   
   @Override
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
   
   @Override
   public void addIncompressibilityStiffness (
      double s, VectorBase intGi, VectorBase intGj) {

      Vector3d intGi3 = (Vector3d) intGi; 
      Vector3d intGj3 = (Vector3d) intGj;
      
      if (FemModel3d.noIncompressStiffnessDamping) {
         if (myKX == null) {
            myKX = new Matrix3d();
         }
         FemUtilities.addIncompressibilityStiffness (myKX, s, intGi3, intGj3);
      }
      else {
         FemUtilities.addIncompressibilityStiffness (myK, s, intGi3, intGj3);
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
   
   @Override
   public void addPressureStiffness( VectorBase gi, double p,
      VectorBase gj, double dv) {
      
      Vector3d gi3 = (Vector3d) gi; 
      Vector3d gj3 = (Vector3d) gj;
      
      if (FemModel3d.noIncompressStiffnessDamping) {
         if (myKX == null) {
            myKX = new Matrix3d();
         }
         FemUtilities.addPressureStiffness (myKX, gi3, p, gj3, dv);  
         FemUtilities.addPressureStiffness (myK, gi3, -p, gj3, dv);  
      }
      
   }
   
   public void addMaterialStiffness(
      double iN, double jN, Vector3d idN, Vector3d jdN, double dv, double t,
      Vector3d[] gct, SymmetricMatrix3d matStress, Matrix6d matTangent, 
      Vector3d gi, Vector3d gj, double p) {
      FemUtilities.addShellMaterialStiffness(
         myK, iN, jN, idN, jdN, dv, t, gct, matStress, matTangent);
      
      addPressureStiffness(gi, p, gj, dv);
   }
}
