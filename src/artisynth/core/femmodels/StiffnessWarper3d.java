/**
 * Copyright (c) 2014, by the Authors: John E Lloyd (UBC)
 *
 * This software is freely available under a 2-clause BSD license. Please see
 * the LICENSE file in the ArtiSynth distribution directory for details.
 */
package artisynth.core.femmodels;

import artisynth.core.materials.FemMaterial;
import artisynth.core.materials.SolidDeformation;
import maspack.matrix.Matrix3d;
import maspack.matrix.Matrix3dBase;
import maspack.matrix.Matrix6d;
import maspack.matrix.RotationMatrix3d;
import maspack.matrix.SVDecomposition3d;
import maspack.matrix.SymmetricMatrix3d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;

/** 
 * Implements stiffness warping for a particular 3d fem element.
 *
 * <p>Note: it is important that all these methods are called from the same
 * (simulation) thread. In particular, they should not be called by methods
 * activated by the GUI.
 */
public class StiffnessWarper3d {
 
   Matrix3d[][] K0;
   Vector3d[] f0;

   // A and R are used to compute stiffness warping. 
   protected RotationMatrix3d R = null;
   protected Matrix3d J0inv = null;
   protected double myConditionNum = 0;

   public StiffnessWarper3d (int numNodes) {
      K0 = new Matrix3d[numNodes][numNodes];
      f0 = new Vector3d[numNodes];

      for (int i=0; i<numNodes; i++) {
         for (int j=0; j<numNodes; j++) {
            K0[i][j] = new Matrix3d();
         }
         f0[i] = new Vector3d();
      }
   }

   public void computeInitialStiffness (FemElement3d e, FemMaterial mat) {

      // reset stiffness and RHS
      for (int i=0; i<e.myNodes.length; i++) {
         for (int j=0; j<e.myNodes.length; j++) {
            K0[i][j].setZero();
         }
         f0[i].setZero();
      }

      IntegrationPoint3d[] ipnts = e.getIntegrationPoints();
      IntegrationData3d[] idata = e.getIntegrationData();

      Matrix6d D = new Matrix6d(); // fill in ...

      SolidDeformation def = new SolidDeformation();
      def.setAveragePressure(0);
      def.setF(Matrix3d.IDENTITY);
      def.setR(Matrix3d.IDENTITY);

      for (int k=0; k<ipnts.length; k++) {

         IntegrationPoint3d pt = ipnts[k];
         IntegrationData3d dt = idata[k];
                  
         double dv0 = dt.myDetJ0*pt.getWeight();
         if (dt.myScaling != 1) {
            dv0 *= dt.myScaling;
         }

         Matrix3d Q = dt.myFrame == null ? Matrix3d.IDENTITY : dt.myFrame;
         Vector3d[] GNx0 = pt.updateShapeGradient(dt.myInvJ0);

         // compute tangent matrix under zero stress
         mat.computeTangent(D, SymmetricMatrix3d.ZERO, def, Q, null);
         
         for (int i=0; i<e.myNodes.length; i++) {
            for (int j=0; j<e.myNodes.length; j++) {
               FemUtilities.addMaterialStiffness (
                  K0[i][j], GNx0[i], D, GNx0[j], dv0);
            }
         }
      }      
      
      // initial RHS
      Vector3d tmp = new Vector3d();
      for (int i=0; i<e.myNodes.length; i++) {
         tmp.setZero();
         for (int j=0; j<e.myNodes.length; j++) {
            K0[i][j].mulAdd (tmp, e.myNodes[j].myRest, tmp);
         }
         f0[i].set (tmp);
      }
   }

   public void computeInitialStiffness (FemElement3d e, double E, double nu) {

      IntegrationPoint3d[] ipnts = e.getIntegrationPoints();
      IntegrationData3d[] idata = e.getIntegrationData();

      Matrix6d D = new Matrix6d(); // fill in ...
      double dia = (1 - nu) / (1 - 2 * nu);
      double off = nu / (1 - 2 * nu);

      D.m00 = dia; D.m01 = off; D.m02 = off;
      D.m10 = off; D.m11 = dia; D.m12 = off;
      D.m20 = off; D.m21 = off; D.m22 = dia;
      D.m33 = 0.5;
      D.m44 = 0.5;
      D.m55 = 0.5;
      D.scale (E/(1+nu));


      for (int i=0; i<e.myNodes.length; i++) {
         for (int j=0; j<e.myNodes.length; j++) {
            K0[i][j].setZero();
         }
         f0[i].setZero();
      }

      for (int k=0; k<ipnts.length; k++) {
         IntegrationPoint3d pt = ipnts[k];
         double dv = idata[k].myDetJ0*pt.getWeight();
         Vector3d[] GNx0 = pt.updateShapeGradient(idata[k].myInvJ0);
         if (idata[k].myScaling != 1) {
            dv *= idata[k].myScaling;
         }
         for (int i=0; i<e.myNodes.length; i++) {
            for (int j=0; j<e.myNodes.length; j++) {
               FemUtilities.addMaterialStiffness (
                  K0[i][j], GNx0[i], D, SymmetricMatrix3d.ZERO, GNx0[j], dv);
            }
         }
      }      
      
      // initial RHS
      Vector3d tmp = new Vector3d();
      for (int i=0; i<e.myNodes.length; i++) {
         tmp.setZero();
         for (int j=0; j<e.myNodes.length; j++) {
            K0[i][j].mulAdd (tmp, e.myNodes[j].myRest, tmp);
         }
         f0[i].set (tmp);
      }
   }

   public void setInitialJ (
      FemNode3d n0, FemNode3d n1, FemNode3d n2, FemNode3d n3) {
      Vector3d tmp = new Vector3d();
      Matrix3d A = new Matrix3d();
      tmp.sub (n1.myRest, n0.myRest);
      A.setColumn (0, tmp);
      tmp.sub (n2.myRest, n0.myRest);
      A.setColumn (1, tmp);
      tmp.sub (n3.myRest, n0.myRest);
      A.setColumn (2, tmp);

      J0inv = new Matrix3d();
      J0inv.invert (A);
      myConditionNum = A.infinityNorm() * J0inv.infinityNorm();
   }

   public double getConditionNum () {
      return myConditionNum;
   }

   public void computeWarping (
      FemNode3d n0, FemNode3d n1, FemNode3d n2, FemNode3d n3) {
      Vector3d tmp = new Vector3d();
      Matrix3d A = new Matrix3d();
      tmp.sub (n1.getLocalPosition(), n0.getLocalPosition());
      A.setColumn (0, tmp);
      tmp.sub (n2.getLocalPosition(), n0.getLocalPosition());
      A.setColumn (1, tmp);
      tmp.sub (n3.getLocalPosition(), n0.getLocalPosition());
      A.setColumn (2, tmp);

      A.mul (J0inv);
      computeRotation (A, null);
   }

   public void computeRotation (Matrix3d F, SymmetricMatrix3d P) {
      if (R == null) {
         R = new RotationMatrix3d();
      }
      SVDecomposition3d SVD = new SVDecomposition3d();
      SVD.polarDecomposition(R, P, F);
   }
   
   public void setRotation(Matrix3dBase R) {
      if (this.R == null) {
         this.R = new RotationMatrix3d();
      }
      this.R.set(R);
   }
   
   public RotationMatrix3d getRotation() {
      return R;
   }

   /**
    * Computes F = RP, R a rotation matrix, P a symmetric matrix
    * @param F matrix to decompose
    * @param R rotational component
    * @param P symmetric component
    */
   public static void computeRotation (Matrix3d F, Matrix3d R, SymmetricMatrix3d P) {
      SVDecomposition3d SVD = new SVDecomposition3d();
      try {
         SVD.factor (F);
      }
      catch (Exception e) {
         System.out.println ("F=\n" + F.toString ("%g"));
         R.setIdentity();
      }
      
      Matrix3d U = SVD.getU();
      Matrix3d V = SVD.getV();
      Vector3d s = new Vector3d();
      SVD.getS(s);

      double detU = U.orthogonalDeterminant();
      double detV = V.orthogonalDeterminant();
      if (detV * detU < 0) { /* then one is negative and the other positive */
         if (detV < 0) { /* negative last column of V */
            V.m02 = -V.m02;
            V.m12 = -V.m12;
            V.m22 = -V.m22;
         }
         else /* detU < 0 */
         { /* negative last column of U */
            U.m02 = -U.m02;
            U.m12 = -U.m12;
            U.m22 = -U.m22;
         }
         s.z = -s.z;
      }
      R.mulTransposeRight (U, V);
      if (P != null) {
         // place the symmetric part in P
         P.mulDiagTransposeRight (V, s);
      }
   }

   public void addNodeStiffness (Matrix3d Kij, int i, int j, boolean warping) {
      if (warping) {
         Matrix3d A = new Matrix3d();
         A.mulTransposeRight (K0[i][j], R);
         A.mul (R, A);
         Kij.add (A);
      }
      else {
         Kij.add (K0[i][j]);
      }
   }

   public void addNodeStiffness (
      FemNodeNeighbor nbr, int i, int j, boolean warping) {
      if (warping) {
         Matrix3d A = new Matrix3d();
         A.mulTransposeRight (K0[i][j], R);
         A.mul (R, A);
         nbr.addStiffness (A);
      }
      else {
         nbr.addStiffness (K0[i][j]);
      }
   }

   public void addNodeForce (
      Vector3d f, int i, FemNode3d[] nodes, boolean warping) {
      Vector3d tmp = new Vector3d();
      if (warping) {
         Vector3d pos = new Vector3d();
         for (int j=0; j<nodes.length; j++) {
            R.mulTranspose (pos, nodes[j].getLocalPosition());
            K0[i][j].mulAdd (tmp, pos, tmp);
         }
         tmp.sub (f0[i]);
         R.mul (tmp, tmp);
         f.add (tmp);
      }
      else {
         for (int j=0; j<nodes.length; j++) {
            K0[i][j].mulAdd (tmp, nodes[j].getLocalPosition(), tmp);
         }
         tmp.sub (f0[i]);
         f.add (tmp);
      }
   }
   
   // required for static analysis
   public void addNodeForce0(Vector3d f, int i, boolean warping) {
      if (warping) {
         Vector3d tmp = new Vector3d();
         R.mul(tmp, f0[i]);
         f.add (tmp);
      } else {
         f.add(f0[i]);
      }
   }
   
   // required for static analysis
   public void addNodeForce0(VectorNd f, int offset, int i, boolean warping) {
      if (warping) {
         Vector3d tmp = new Vector3d();
         R.mul(tmp, f0[i]);
         f.set(offset, f.get(offset) + tmp.x);
         f.set(offset+1, f.get(offset+1) + tmp.y);
         f.set(offset+2, f.get(offset+2) + tmp.z);
      } else {
         f.set(offset, f.get(offset) + f0[i].x);
         f.set(offset+1, f.get(offset+1) + f0[i].y);
         f.set(offset+2, f.get(offset+2) + f0[i].z);
      }
   }

   public RotationMatrix3d getR() {
      return R;
   }
}
