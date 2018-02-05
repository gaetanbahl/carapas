/**
 * Copyright (c) 2014, by the Authors: Antonio Sanchez (UBC)
 *
 * This software is freely available under a 2-clause BSD license. Please see
 * the LICENSE file in the ArtiSynth distribution directory for details.
 */
package artisynth.core.mfreemodels;

import artisynth.core.femmodels.FemNodeNeighbor;
import artisynth.core.femmodels.FemUtilities;
import artisynth.core.femmodels.IntegrationData3d;
import artisynth.core.materials.FemMaterial;
import artisynth.core.materials.SolidDeformation;
import maspack.matrix.Matrix3d;
import maspack.matrix.Matrix3dBase;
import maspack.matrix.Matrix6d;
import maspack.matrix.RotationMatrix3d;
import maspack.matrix.SVDecomposition3d;
import maspack.matrix.SymmetricMatrix3d;
import maspack.matrix.Vector3d;

/** 
 * Implements stiffness warping for a particular integration region.
 *
 * <p>Note: it is important that all these methods are called from the same
 * (simulation) thread. In particular, they should not be called by methods
 * activated by the GUI.
 */
public class MFreeStiffnessWarper3d {
 
   Matrix3d[][] K0;
   MFreeElement3d myElem;
   
   // A and R are used to compute stiffness warping. 
   protected Matrix3d A = new Matrix3d();
   protected RotationMatrix3d R = new RotationMatrix3d();
   protected Vector3d tmp = new Vector3d();
   protected Vector3d pos = new Vector3d();
   protected SVDecomposition3d SVD = new SVDecomposition3d();

   protected double myConditionNum = 0;

   public MFreeStiffnessWarper3d (int numNodes) {
      K0 = new Matrix3d[numNodes][numNodes];
      
      for (int i=0; i<numNodes; i++) {
         for (int j=0; j<numNodes; j++) {
            K0[i][j] = new Matrix3d();  
         }
      }
   }

   public void computeInitialStiffness (MFreeElement3d e, FemMaterial mat) {

      // reset stiffness and RHS
      for (int i=0; i<e.myNodes.length; i++) {
         for (int j=0; j<e.myNodes.length; j++) {
            K0[i][j].setZero();
         }
         // f0[i].setZero();
      }
      
      MFreeIntegrationPoint3d[] ipnts = e.getIntegrationPoints();
      IntegrationData3d[] idata = e.getIntegrationData();
      myElem = e;
      
      SolidDeformation def = new SolidDeformation();
      def.setAveragePressure(0);
      def.setF(Matrix3d.IDENTITY);
      def.setR(Matrix3d.IDENTITY);

      Matrix6d D = new Matrix6d(); // fill in ...
      
      for (int k=0; k<ipnts.length; k++) {
         MFreeIntegrationPoint3d pt = ipnts[k];
         IntegrationData3d idat = idata[k];
         double dv = pt.getWeight()*idat.getDetJ0();
         Vector3d[] GNx = pt.updateShapeGradient(idat.getInvJ0());
         
         Matrix3d Q = idat.getFrame() == null ? Matrix3d.IDENTITY : idat.getFrame();
         
         // compute tangent matrix under zero stress
         mat.computeTangent(D, SymmetricMatrix3d.ZERO, def, Q, null);
         
         for (int i=0; i<e.myNodes.length; i++) {
            for (int j=0; j<e.myNodes.length; j++) {
               Matrix3d KA = new Matrix3d();
               
               FemUtilities.addMaterialStiffness (
                  KA, GNx[i], D, SymmetricMatrix3d.ZERO, GNx[j], dv);
               K0[i][j].add(KA);
               
            }
         }
      }      
      
      //      // initial RHS
      //      Vector3d tmp = new Vector3d();
      //      for (int i=0; i<e.myNodes.length; i++) {
      //         tmp.setZero();
      //         for (int j=0; j<e.myNodes.length; j++) {
      //            K0[i][j].mulAdd (tmp, e.myNodes[j].getRestPosition(), tmp);
      //            //             if (e.getNumber() == 0 && i<4 && j<4) {
      //            //                System.out.println ("K0["+i+"]["+j+"]");
      //            //                System.out.println (K0[i][j].toString("%10.5f"));
      //            //             }
      //         }
      //         f0[i].set (tmp);
      //      }
   }
   
   public Matrix3d[][] getInitialStiffnesses() {
      return K0;
   }
   
   public RotationMatrix3d getRotation() {
      return R;
   }

   public void computeRotation (Matrix3d F, SymmetricMatrix3d P) {
      if (R == null) {
         R = new RotationMatrix3d();
      }
      SVDecomposition3d SVD = new SVDecomposition3d();
      SVD.polarDecomposition(R, P, F);
   }

   public void addNodeStiffness(Matrix3d Kij, boolean[][] active, int i, int j, boolean warping) {
      
      if (active[i][j]) {
         if (warping) {
            A.mulTransposeRight (K0[i][j], R);
            A.mul (R, A);
            Kij.add (A);
         }
         else {
            Kij.add (K0[i][j]);
         }
      }
   }
   
   public void addNodeStiffness (FemNodeNeighbor nbr, int i, int j, boolean warping) {
      
      // if (active[i][j]) {
         if (warping) {
            A.mulTransposeRight (K0[i][j], R);
            A.mul (R, A);
            nbr.addStiffness (A);
         }
         else {
            
//            String pntStr = "stiff(" + (myElem.myNodes[i].getNumber()+1) + "," + (myElem.myNodes[j].getNumber()+1) + "," + (myElem.getNumber()+1) + ","+
//               (myElem.getGrandParent().getNumber() + 1) + ")";
//            System.out.println(pntStr + ".K = [" + K0[i][j] + "];");
//            System.out.println(pntStr + ".nbr = [" + (myElem.myNodes[i].getNumber()+1) + "," +(nbr.getNode().getNumber()+1) + "];");
            
            nbr.addStiffness (K0[i][j]);
            
         }
      // }
   }

   public void addNodeForce (
      Vector3d f, int i, MFreeNode3d[] nodes, boolean warping, FemNodeNeighbor[] nbr) {

      if (warping) {
         tmp.setZero();
         //R.setIdentity();
         for (int j=0; j<nodes.length; j++) {
            // if (active[i][j]) { 
               R.mulTranspose (pos, nodes[j].getFalsePosition());
               pos.sub(nodes[j].getRestPosition());
               // R.mulTranspose (pos, nodes[j].getFalseDisplacement());
               K0[i][j].mulAdd (tmp, pos, tmp);
            // }
         }
         R.mul (tmp, tmp);
         f.add (tmp);
         
      }
      else {
         tmp.setZero();
         for (int j=0; j<nodes.length; j++) {
            // if (active[i][j]) {
               Vector3d tmpF = new Vector3d();
               nodes[j].getFalseDisplacement(pos);
               K0[i][j].mulAdd (tmp, pos, tmp);
               K0[i][j].mul(tmpF,pos);
               
            // }
         }
         f.add (tmp);
      }
   }

   public void setRotation(Matrix3dBase R) {
      if (this.R == null) {
         this.R = new RotationMatrix3d();
      }
      this.R.set(R);
   }

}
