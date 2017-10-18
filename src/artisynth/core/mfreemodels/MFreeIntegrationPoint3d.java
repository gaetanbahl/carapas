/**
 * Copyright (c) 2014, by the Authors: Antonio Sanchez (UBC)
 *
 * This software is freely available under a 2-clause BSD license. Please see
 * the LICENSE file in the ArtiSynth distribution directory for details.
 */
package artisynth.core.mfreemodels;

import java.util.ArrayList;
import java.util.Arrays;

import artisynth.core.femmodels.IntegrationPoint3d;
import artisynth.core.mechmodels.PointState;
import maspack.matrix.Matrix3d;
import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;

public class MFreeIntegrationPoint3d extends IntegrationPoint3d implements MFreePoint3d {

   MFreeNode3d[] myDependentNodes;
   PointState myState;
   Point3d myRestPosition;
   int myID;

   public MFreeIntegrationPoint3d() {
      super(0);
      myState = new PointState();
      myRestPosition = new Point3d();
      myID = -1;
     
   }

   public int getID() {
      return myID;
   }

   public void setID(int num) {
      myID = num;
   }

   public MFreeIntegrationPoint3d(MFreeNode3d[] deps, VectorNd coords) {
      this();
      setDependentNodes(deps,coords);
   }

   public MFreeNode3d[] getDependentNodes() {
      return myDependentNodes;
   }

   public void setDependentNodes(MFreeNode3d[] nodes, VectorNd coords) {
      myDependentNodes = Arrays.copyOf(nodes, nodes.length);
      super.init(myDependentNodes.length, 1);
      setPressureWeights(new VectorNd(new double[]{1}));  // XXX default pressure weights?
      setNodeCoordinates(coords);
      updatePosAndVelState();
   }

   public Point3d getPosition() {
      return myState.getPos();
   }

   public Point3d getRestPosition() {
      return myRestPosition;
   }

   public VectorNd getNodeCoordinates() {
      return getShapeWeights();
   }

   public int getNodeCoordIdx(MFreeNode3d node) {
      for (int i=0; i<myDependentNodes.length; ++i) {
         if (node == myDependentNodes[i]) {
            return i;
         }
      }
      return -1;
   }

   public double getShapeCoordinate(MFreeNode3d node) {
      int idx = getNodeCoordIdx(node);
      if (idx < 0) {
         return 0;
      }
      return getShapeWeights().get(idx);
   }

   @Override 
   public Vector3d getCoords() {
      // meaningless
      return null;
   }

   public void setNodeCoordinates(VectorNd coords) {

      setShapeWeights(coords);
      updateRestPosition();
      updatePosState();
      updateVelState();
   }

   public void updatePosState() {
      myState.setPos(Point3d.ZERO);
      for (int i=0; i<myDependentNodes.length; i++) {
         myState.scaledAddPos(N.get(i),myDependentNodes[i].getFalsePosition());
      }
   }

   public void updateVelState() {
      myState.setVel(Vector3d.ZERO);
      for (int i=0; i<myDependentNodes.length; i++) {
         myState.scaledAddVel(N.get(i),myDependentNodes[i].getFalseVelocity());
      }
   }

   public void updatePosAndVelState() {
      updatePosState();
      updateVelState();
   }

   /** 
    * Create an integration point for a given element.
    */
   public static MFreeIntegrationPoint3d create (MFreeNode3d[] dependentNodes, VectorNd shapeN, ArrayList<Vector3d> shapeGrad, double w) {

      int nnodes = dependentNodes.length;
      MFreeIntegrationPoint3d ipnt = new MFreeIntegrationPoint3d(dependentNodes, shapeN);
      ipnt.setWeight(w);

      for (int i=0; i<nnodes; i++) {
         ipnt.setShapeGrad (i, shapeGrad.get(i));
      }
      return ipnt;
   }

   public void computeJacobian () {
      myJ.setZero();
      for (int i=0; i<myDependentNodes.length; i++) {
         Point3d pos = myDependentNodes[i].getFalsePosition();
         Vector3d dNds = GNs[i];
         myJ.addOuterProduct (pos.x, pos.y, pos.z, 
            dNds.x, dNds.y, dNds.z);
      }

   }

   public void computeJacobianAndGradient (Matrix3d invJ0) {

      myJ.setZero();
      for (int i=0; i<myDependentNodes.length; i++) {
         Vector3d pos = myDependentNodes[i].getFalsePosition();
         Vector3d dNds = GNs[i];
         myJ.addOuterProduct (pos.x, pos.y, pos.z, dNds.x, dNds.y, dNds.z);
      }

      if (invJ0 != null) {
         F.mul (myJ, invJ0);
      } else {
         F.set(myJ);
      }      
      detF = F.determinant();
   }

   public void computeJacobianAndGradient() {
      computeJacobianAndGradient(null);
   }

   public void computeGradientForRender (Matrix3d Fmat, 
      Matrix3d invJ0) {

      // compute J in Fmat
      Fmat.setZero();
      for (int i=0; i<myDependentNodes.length; i++) {
         Point3d pos = myDependentNodes[i].getFalsePosition();
         Vector3d dNds = GNs[i];
         Fmat.addOuterProduct (pos.x,pos.y,pos.z, dNds.x, dNds.y, dNds.z);
      }      
      if (invJ0 != null) {
         Fmat.mul (invJ0);
      }
   }

   public void computeGradientForRender (Matrix3d Fmat) {
      computeGradientForRender(Fmat, null);
   }

   public void updateRestPosition() {
      myRestPosition.setZero();
      for (int i=0; i<myDependentNodes.length; i++) {
         myRestPosition.scaledAdd(N.get(i), myDependentNodes[i].getRestPosition());
      }
   }

   public void computePosition (Point3d pos) {
      double[] Nbuf = N.getBuffer();
      for (int i=0; i<myDependentNodes.length; i++) {
         pos.scaledAdd (Nbuf[i], myDependentNodes[i].getFalsePosition());
      }
   }

   public double getDetJ() {
      return myJ.determinant();
   }

   public boolean reduceDependencies(double tol) {

      int ndeps = 0;
      boolean changed = false;
      for (int i=0; i<myDependentNodes.length; i++) {
         if (Math.abs(N.get(i)) <= tol) {
            changed = true;
            N.set(i, 0);
         } else {
            if (changed) {
               myDependentNodes[ndeps] = myDependentNodes[i];
               N.set(ndeps, N.get(i));
            }
            ++ndeps;
         }
      }
      if (changed) {
         myDependentNodes = Arrays.copyOf(myDependentNodes, ndeps);
         N.setSize(ndeps);
         N.scale(1.0/N.sum()); // re-sum to one   
      }

      return changed;
   }
   
   @Override
   public void setNumber(int num) {
      super.setNumber(num);
   }


}
