/**
 * Copyright (c) 2014, by the Authors: Antonio Sanchez (UBC)
 *
 * This software is freely available under a 2-clause BSD license. Please see
 * the LICENSE file in the ArtiSynth distribution directory for details.
 */
package artisynth.core.mfreemodels;

import maspack.matrix.Vector3d;
import maspack.matrix.VectorBase;
import artisynth.core.femmodels.FemNodeNeighbor;

public class MFreeNodeNeighbor extends FemNodeNeighbor {

   MFreeNode3d myMNode;
   
   public MFreeNodeNeighbor(MFreeNode3d node) {
      super(node);
      myMNode = node;
   }
   
   public MFreeNode3d getMFreeNode() {
      return myMNode;
   }
   
   @Override
   public void addDampingForce (VectorBase fd) {
      Vector3d fd3 = (Vector3d) fd;
      fd3.mulAdd (myK, myMNode.getFalseVelocity(), fd3);
   }
   
}
