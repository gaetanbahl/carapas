package artisynth.core.femmodels;

import java.awt.Color;
import java.util.LinkedList;

import artisynth.core.femmodels.ShellIntegrationPoint3d.NODE_POS;
import artisynth.core.materials.FemMaterial;
import artisynth.core.materials.IncompressibleMaterial;
import artisynth.core.materials.LinearMaterial;
import artisynth.core.materials.SolidDeformation;
import artisynth.core.materials.ViscoelasticBehavior;
import artisynth.core.materials.ViscoelasticState;
import maspack.matrix.Matrix3d;
import maspack.matrix.Matrix6d;
import maspack.matrix.Matrix6dBlock;
import maspack.matrix.MatrixBlock;
import maspack.matrix.SparseNumberedBlockMatrix;
import maspack.matrix.SymmetricMatrix3d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;
import maspack.render.Renderer;

public class ShellFemModel3d extends FemModel3d {

   public ShellFemModel3d () {
      this(null);
   }
   
   public ShellFemModel3d (String name) {
      super(name);
   }
   
   @Override
   public void addElement(FemElement3d newEle) {
      // Create a new list of the existing elements + new element.
      // This list will then be passed immediately to 
      // refreshNodeDirectors(newList).
      // This is done because we cannot call
      // refreshNodeDirectors(this.myElements) after super.addElement(newEle)
      // due to the fact that super.addElement(newEle) computes volume()
      // which in turn requires knowing the node directors.
      LinkedList<ShellFemElement3d> eles = new LinkedList<ShellFemElement3d>();
      for (FemElement3d e : this.myElements) {
         eles.add((ShellFemElement3d)e);
      }
      newEle.setIndex ( eles.size() );
      eles.add((ShellFemElement3d)newEle);
      
      // Use this new list to compute each node's director
      refreshNodeDirectors0(eles);
      
      super.addElement (newEle);
   }
   
   /**
    * Compute the rest director vector for each node of this model. This should
    * be invoked after the nodes and elements are added to this model.
    * 
    * Implementation is simply finding the vertex normal for each node.
    * 
    * FEBio: FEMesh::InitShellsNew().
    * 
    * Postcond:
    * node.myDirector0 is initialized for shell element node.
    * 
    * @param eles
    * List of all the shell elements in the shell model.
    */
   protected void refreshNodeDirectors0(LinkedList<ShellFemElement3d> eles) {
      for (FemElement3d e : eles) {
         for (FemNode3d n : e.myNodes) {
            ShellFemNode3d sn = (ShellFemNode3d) n;
            sn.myDirector0.setZero ();
         }
      }
      
      for (FemElement3d ele : eles) {
         ShellFemElement3d sEle = (ShellFemElement3d) ele;
         for (int i = 0; i < sEle.numNodes(); i++) {     
            // Get next and prev nodes relative to i-th node.
            int n = (i+1) % sEle.numNodes();
            int p = (i==0) ? sEle.numNodes()-1 : i-1; 
            
            Vector3d iPos = sEle.myNodes[i].getPosition ();
            Vector3d nPos = sEle.myNodes[n].getPosition ();
            Vector3d pPos = sEle.myNodes[p].getPosition ();
            
            Vector3d n_i = new Vector3d();
            n_i.sub (nPos, iPos);
            
            Vector3d p_i = new Vector3d();
            p_i.sub (pPos, iPos);
            
            Vector3d dir = new Vector3d();
            dir.cross (n_i, p_i);
            dir.normalize ();
            dir.scale (sEle.getShellThickness());
            
            ShellFemNode3d sn = (ShellFemNode3d) sEle.myNodes[i];
            sn.myDirector0.add(dir);
         }
      }
      
      // Average the directors.
      for (FemNode3d n : myNodes) {
         ShellFemNode3d sn = (ShellFemNode3d) n;
         
         if (sn.myAdjElements.size() == 0) {
            throw new RuntimeException("Node has no adjacent elements. "
            + "Did you forget to call node.myAdjElements(this) in the element "
            + "constructor?");
         }
         
         sn.myDirector0.scale(1.0/sn.myAdjElements.size());
      }
   }
   
   
   /**
    * Compute the force of each shell node due to stress, stiffness,
    * and damping.
    * 
    * Postcond:
    * node.myForce and node.myDirForce are set for each node.
    */
   @Override
   protected void updateNodeForces(double t) {
      if (!myStressesValidP) {
         updateStressAndStiffness();
      }
      
      boolean hasGravity = !myGravity.equals(Vector3d.ZERO);
      
      VectorNd fk6 = new VectorNd(6); // stiffness force
      VectorNd fd6 = new VectorNd(6); // damping force
      VectorNd md6 = new VectorNd(6); // mass damping (used with attached frames)

      for (FemNode3d n : myNodes) {
         ShellFemNode3d sn = (ShellFemNode3d) n;
         
         /* Get node velocity */
         VectorNd v6 = new VectorNd(sn.getVelStateSize());
         sn.getVelocity(v6);
         
         // n.setForce (n.getExternalForce());
         if (hasGravity) {
            sn.addScaledForce(n.getMass(), myGravity);
         }
         
         // Internal force already computed from updateStressAndStiffness().
         sn.getInternalForce(fk6);
         fd6.setZero();
         
         if (myStiffnessDamping != 0) {    
            for (NodeNeighbor nbr : getNodeNeighbors(n)) {
               nbr.addDampingForce(fd6);
            }
            // used for soft nodal-based incompressibilty:
            for (NodeNeighbor nbr : getIndirectNeighbors(n)) {
               nbr.addDampingForce(fd6);
            }
            fd6.scale(myStiffnessDamping);
         }
         if (usingAttachedRelativeFrame()) {    // SKIPPED
            md6.scale (myMassDamping * n.getMass(), v6);
            sn.subForce (md6);
            // if (n.isActive()) {
            //    myFrame.addPointForce (n.getLocalPosition(), n.getForce());
            // }
            fk6.add (fd6);
            
            // DANNY: TODO commented out
            //fk6.transform (myFrame.getPose().R);
            //n.subLocalForce (fk6);
            throw new RuntimeException("Unimplemented");
            
            //if (n.isActive()) {
               // myFrame.addPointForce (n.getLocalPosition(), n.getForce());
               //}
         }
         else {
            fd6.scaledAdd(myMassDamping * sn.getMass(), v6, fd6);     
            sn.subForce(fk6);                   
            sn.subForce(fd6);                   
         }
      }
   }
   
   
   @Override
   public void updateStressAndStiffness() {
      // Reset internal direction force which isn't done in 
      // super.updateStressAndStiffness().
      for (FemNode3d n : myNodes) {
         ShellFemNode3d sn = (ShellFemNode3d) n;
         sn.myInternalDirForce.setZero();
      }
      
      /* At this point, all the node positions and directions have updated 
       * from the previous timestep, so we need to update the co and contra
       * vectors for each integration point. This is required before calling 
       * any subsequent integration point method in this timestep. */
      for (FemElement3d ele : this.getElements ()) {
         for (IntegrationPoint3d iPt : ele.getIntegrationPoints()) {
            ((ShellIntegrationPoint3d)iPt).updateCoContraVectors();
         }
      }
      
      super.updateStressAndStiffness();
   }
   
   
   /**
    * Compute the force and neighbor stiffness for each node of a shell
    * element.
    * 
    * Postcond:
    * node.myInternalForce and node.myInternalDirForce are incremented.
    * nodeNeighbor[i][j].myK is incremented where (i,j) is every possible
    * node neighbor pair between the shell element nodes.
    */
   @Override
   protected void computeNonlinearStressAndStiffness(
      FemElement3d ele, FemMaterial mat, Matrix6d D, IncompMethod softIncomp) {

      ShellFemElement3d e = ((ShellFemElement3d)ele);
      
      ShellIntegrationPoint3d[] ipnts = e.getIntegrationPoints();
      ShellIntegrationData3d[] idata = e.getIntegrationData();
      FemNode3d[] nodes = e.getNodes();
      int npvals = e.numPressureVals();
      double pressure = 0; // pressure for incompressibility
      double vol = 0;
      double restVol = 0;
      IncompressibleMaterial imat = null;
      Vector3d[] avgGNx = null;
      MatrixBlock[] constraints = null;
      double[] nodalExtrapMat = null;
      SymmetricMatrix3d C = new SymmetricMatrix3d();

      ViscoelasticBehavior veb = mat.getViscoBehavior();
      double vebTangentScale = 1;
      if (veb != null) {                // SKIPPED
         vebTangentScale = veb.getTangentScale();
      }

      SymmetricMatrix3d sigmaAux = null;
      Matrix6d DAux = null;
      if (e.numAuxiliaryMaterials() > 0) {              // SKIPPED
         sigmaAux = new SymmetricMatrix3d();
         DAux = new Matrix6d();
      }

      // see if material is linear
      boolean corotated = false;
      ShellIntegrationPoint3d wpnt = null;
      LinearMaterial linMat = null;
      if (mat instanceof LinearMaterial) {      // SKIPPED with neoHook material

         linMat = (LinearMaterial)mat;
         corotated = linMat.isCorotated();
         wpnt = e.getWarpingPoint();
         ShellIntegrationData3d data = e.getWarpingData();
         wpnt.computeJacobianAndGradient();       
         wpnt.sigma.setZero();
         if (corotated) {
            e.computeWarping(wpnt.F, myEps);
         }
         else {
            myEps.setSymmetric(wpnt.F);
         }
         // compute Cauchy strain
         myEps.m00 -= 1;
         myEps.m11 -= 1;
         myEps.m22 -= 1;
      }

      e.setInverted(false); // will check this below
      vol = e.getVolume();
      // SKIPPED
      if (mat.isIncompressible() && softIncomp != IncompMethod.NODAL) {
         imat = (IncompressibleMaterial)mat;
         if (softIncomp == IncompMethod.ELEMENT) {

            computePressuresAndRinv (e, imat, vebTangentScale);
            if (D != null) {
               constraints = e.getIncompressConstraints();
               for (int i = 0; i < e.myNodes.length; i++) {
                  constraints[i].setZero();
               }
            }
            restVol = e.getRestVolume();
         }
      }
      // SKIPPED
      else if (softIncomp == IncompMethod.NODAL) {
         if (e instanceof ShellQuadElement) {
            // DANNY TODO
            //((ShellQuadElement)e).getAreaWeightedNormals(myNodalConstraints);
            for (int i = 0; i < 4; i++) {
               myNodalConstraints[i].scale(-1 / 12.0);
            }
         }
         else {
            for (int i = 0; i < e.numNodes(); i++) {
               myNodalConstraints[i].setZero();
            }
         }

      }

      // SKIPPED
      if (linMat != null) {
         for (int i = 0; i < nodes.length; i++) {
            int bi = nodes[i].getSolveIndex();
            if (bi != -1) {
               FemNode3d n = nodes[i];
               if (!myStiffnessesValidP) {
                  for (int j = 0; j < nodes.length; j++) {
                     int bj = nodes[j].getSolveIndex();
                     if (!mySolveMatrixSymmetricP || bj >= bi) {
                        e.addNodeStiffness(i, j, corotated);
                     }
                  }
               }
               e.addNodeForce(n.myInternalForce, i, corotated);
            }
         }
      }

      // SKIPPED
      if (myComputeNodalStress || myComputeNodalStrain) {
         nodalExtrapMat = e.getNodalExtrapolationMatrix();
         if (linMat != null) {
            linMat.addStress(wpnt.sigma,
               myEps, corotated ? e.myWarper.R : null);
            for (int i = 0; i < nodes.length; i++) {
               FemNode3d nodei = nodes[i];
               if (myComputeNodalStress) {
                  nodei.addScaledStress(
                     1.0 / nodei.numAdjacentElements(), wpnt.sigma);
               }
               if (myComputeNodalStrain) {
                  nodei.addScaledStrain(
                     1.0 / nodei.numAdjacentElements(), myEps);
               }
            }
         }
      }

      double[] pbuf = myPressures.getBuffer();
      // e.myAvgStress.setZero();
      if (linMat == null || e.numAuxiliaryMaterials() > 0) {

         SolidDeformation def = new SolidDeformation();
         for (int k = 0; k < ipnts.length; k++) {
            ShellIntegrationPoint3d pt = ipnts[k];
            ShellIntegrationData3d dt = idata[k];
            pt.computeJacobianAndGradient();           
            def.setF(pt.F);
            double detJ = pt.computeInverseJacobian();
            if (detJ < myMinDetJ) {
               myMinDetJ = detJ;
               myMinDetJElement = e;
            }
            // SKIPPED
            if (detJ <= 0 && !e.materialsAreInvertible()) {
               e.setInverted(true);
               myNumInverted++;
            }
            
            double t = pt.coords.z; 
            double dv = detJ * pt.getWeight();
            Vector3d[] GNx = pt.updateShapeGradient(pt.myInvJ);
            Vector3d[] gct = pt.getContraBaseVectors(NODE_POS.CURRENT);
            VectorNd Ns = pt.getShapeWeights ();
            Vector3d[] dNs = pt.getGNs();
            
            // compute pressure
            pressure = 0;
            double[] H = null;

            if (softIncomp == IncompMethod.ELEMENT) {
               H = pt.getPressureWeights().getBuffer();
               for (int l = 0; l < npvals; l++) {
                  pressure += H[l] * pbuf[l];
               }
            }
            // SKIPPED
            else if (softIncomp == IncompMethod.NODAL) {
               if (e instanceof ShellQuadElement) {
                  // use the average pressure for all nodes
                  pressure = 0;
                  for (int i = 0; i < nodes.length; i++) {
                     pressure += nodes[i].myPressure;
                  }
                  pressure /= nodes.length;
               }
               else {
                  pressure = nodes[k].myPressure;
               }
            }
            // SKIPPED
            else if (softIncomp == IncompMethod.FULL && imat != null) {
               pressure = imat.getEffectivePressure(detJ / dt.getDetJ0());
            }

            Matrix3d Q = (dt.myFrame != null ? dt.myFrame : Matrix3d.IDENTITY);

            pt.avgp = pressure;
            def.setAveragePressure(pressure);
            double scaling = dt.myScaling;
            // SKIPPED
            if (linMat != null) {
               pt.sigma.setZero();
               if (D != null) {
                  D.setZero();
               }
            } else {
               mat.computeStress(pt.sigma, def, Q, null);
               // SKIPPED
               if (scaling != 1) {
                  pt.sigma.scale (scaling);
               }
               if (D != null) {
                  mat.computeTangent(D, pt.sigma, def, Q, null);
                  // SKIPPED
                  if (scaling != 1) {
                     D.scale (scaling);
                  }
               }
            }

            // reset pressure to zero for auxiliary
            pt.avgp = 0;
            def.setAveragePressure(0);
            // SKIPPED
            if (e.numAuxiliaryMaterials() > 0) {
               for (AuxiliaryMaterial aux : e.myAuxMaterials) {
                  aux.computeStress(sigmaAux, def, pt, dt, mat);
                  pt.sigma.add(sigmaAux);
                  if (D != null) {
                     aux.computeTangent(DAux, sigmaAux, def, pt, dt, mat);
                     D.add(DAux);
                  }
               }
            }
            pt.avgp = pressure; // bring back pressure term
            def.setAveragePressure(pressure);
            // SKIPPED
            if (veb != null) {
               ViscoelasticState state = idata[k].getViscoState();
               if (state == null) {
                  state = veb.createState();
                  idata[k].setViscoState(state);
               }
               veb.computeStress(pt.sigma, state);
               if (D != null) {
                  veb.computeTangent(D, state);
               }
            }
            else {
               dt.clearState();
            }
            //System.out.println ("sigma=\n" + pt.sigma);

            // pt.avgp += e.myLagrangePressure;

            // e.myAvgStress.scaledAdd (dv, pt.sigma);

            for (int i = 0; i < e.myNodes.length; i++) {
               ShellFemNode3d nodei = (ShellFemNode3d) e.myNodes[i];
               int bi = nodei.getSolveIndex();
               
               // Add stress (pt.sigma) to node force
               FemUtilities.addShellStressForce(
                  nodei.myInternalForce, nodei.myInternalDirForce,
                  pt.sigma, t, dv, Ns.get(i), dNs[i].x, dNs[i].y, gct);

               if (D != null) {
                  double p = 0;
                  double kp = 0;
                  // SKIPPED
                  if (mat.isIncompressible() &&
                      softIncomp != IncompMethod.NODAL) {
                     if (softIncomp == IncompMethod.ELEMENT) {
                        FemUtilities.addToIncompressConstraints(
                           constraints[i], H, GNx[i], dv);
                     }
                     else if (softIncomp == IncompMethod.FULL) {
                        double dV = dt.getDetJ0() * pt.getWeight();
                        kp = imat.getEffectiveModulus(detJ / dt.getDetJ0()) * dV;
                     }
                     p = pressure;
                  }
                  // SKIPPED
                  else if (softIncomp == IncompMethod.NODAL) {
                     if (e.integrationPointsMapToNodes()) {
                        myNodalConstraints[i].scale(dv, GNx[i]);
                     }
                     else { // tet element
                        for (NodeNeighbor nbr : getNodeNeighbors(nodei)) {
                           int j = e.getLocalNodeIndex(nbr.getNode());
                           if (j != -1) {
                              nbr.myDivBlk.scaledAdd(1, myNodalConstraints[j]);
                           }
                        }
                     }
                  }
                  if (bi != -1) {
                     for (int j = 0; j < e.myNodes.length; j++) {
                        int bj = e.myNodes[j].getSolveIndex();
                        if (!mySolveMatrixSymmetricP || bj >= bi) {
                           
                           double iN = Ns.get(i);
                           double jN = Ns.get(j);
                           
                           Vector3d idN = dNs[i];
                           Vector3d jdN = dNs[j];
                           
                           /* Add shell-specific material stiffness */
                           ((ShellFemNodeNeighbor)e.myNbrs[i][j]).
                              addMaterialStiffness (
                              iN, jN, idN, jdN, dv, t, gct, 
                              /*material stress=*/ pt.sigma, 
                              /*material tangent=*/ D,
                              GNx[i], GNx[j], p);

                           if (kp != 0) {               // SKIPPED
                              e.myNbrs[i][j].addDilationalStiffness(
                                 vebTangentScale*kp, GNx[i], GNx[j]);
                           }
                        }
                     }
                  }
               }
               if (nodalExtrapMat != null) {    // SKIPPED
                  if (myComputeNodalStress) {
                     double a = nodalExtrapMat[i * ipnts.length + k];
                     if (a != 0) {
                        nodei.addScaledStress(
                           a / nodei.numAdjacentElements(), pt.sigma);
                     }
                  }
                  if (myComputeNodalStrain) {
                     double a = nodalExtrapMat[i * ipnts.length + k];
                     if (a != 0) {
                        // pt.computeRightCauchyGreen(C);
                        def.computeRightCauchyGreen(C);
                        C.m00 -= 1;
                        C.m11 -= 1;
                        C.m22 -= 1;
                        C.scale(0.5);
                        nodei.addScaledStrain(
                           a / nodei.numAdjacentElements(), C);
                     }
                  }
               }
            }
            // SKIPPED
            if (D != null &&
            softIncomp == IncompMethod.NODAL &&
            e.integrationPointsMapToNodes()) {
               for (NodeNeighbor nbr : getNodeNeighbors(e.myNodes[k])) {
                  int j = e.getLocalNodeIndex(nbr.getNode());
                  if (j != -1) {
                     nbr.myDivBlk.scaledAdd(1, myNodalConstraints[j]);
                  }
               }
            }
         }
      }
      // SKIPPED
      if (D != null) {
         if (mat.isIncompressible() && softIncomp == IncompMethod.ELEMENT) {
            boolean kpIsNonzero = false;
            for (int l = 0; l < npvals; l++) {
               double Jpartial = e.myVolumes[l] / e.myRestVolumes[l];
               myKp[l] =
               imat.getEffectiveModulus(Jpartial) / e.myRestVolumes[l];
               if (myKp[l] != 0) {
                  kpIsNonzero = true;
               }
            }
            // double kp = imat.getEffectiveModulus(vol/restVol)/restVol;
            if (true) {
               for (int i = 0; i < e.myNodes.length; i++) {
                  int bi = e.myNodes[i].getSolveIndex();
                  if (bi != -1) {
                     for (int j = 0; j < e.myNodes.length; j++) {
                        int bj = e.myNodes[j].getSolveIndex();
                        if (!mySolveMatrixSymmetricP || bj >= bi) {
                           e.myNbrs[i][j].addDilationalStiffness(
                              myRinv, constraints[i], constraints[j]);
                        } // end filling in symmetric
                     } // end filling in dilatational stiffness
                  } // end checking if valid index
               } // end looping through nodes
            } // XXX ALWAYS??
         } // end soft elem incompress
      } // end checking if computing tangent
   }

   

   
   /*** Methods pertaining to the mass and solve blocks ***/
   
   
   /**
    * Add the position jacobian to the blocks of the global solve matrix. Each
    * block corresponds to a node neighbor (nbr) with respect to an observing 
    * node (i). This (i,nbr) node pair have their own respective position 
    * jacobian.
    * 
    * Method is overridden to be compatible with 6x6 blocks.
    */
   @Override
   public void addPosJacobian(SparseNumberedBlockMatrix M, double s) {
      if (!myStressesValidP || !myStiffnessesValidP) {
         updateStressAndStiffness();
      }
      for (int i = 0; i < myNodes.size(); i++) {
         FemNode3d node = myNodes.get(i);
         if (node.getSolveIndex() != -1) {
            for (NodeNeighbor nbr : getNodeNeighbors(node)) {
               if (nbr.getNode().getSolveIndex() != -1) {
                  Matrix6dBlock blk =
                  (Matrix6dBlock)M.getBlockByNumber(nbr.myBlkNum);
                  nbr.addPosJacobian(blk, s);
               }
            }
            // used for soft nodal-based incompressibilty:
            for (NodeNeighbor nbr : getIndirectNeighbors(node)) {
               if (nbr.getNode().getSolveIndex() != -1) {
                  Matrix6dBlock blk =
                  (Matrix6dBlock)M.getBlockByNumber(nbr.myBlkNum);
                  nbr.addPosJacobian(blk, s);
               }
            }
         }
      }
      // System.out.println ("symmetric=" + mySolveMatrix.isSymmetric(1e-6));
   }
   
   
   /**
    * Allocate and assign a block from the global solve matrix to a 
    * particular node neighbor.
    * 
    * Method is overridden to be compatible with 6x6 blocks.
    */
   @Override
   protected void addNodeNeighborBlock(
      SparseNumberedBlockMatrix S, NodeNeighbor nbr, int bi) {
      int bj = nbr.getNode().getSolveIndex();
      Matrix6dBlock blk = null;
      int blkNum = -1;
      if (bj != -1) {
         blk = (Matrix6dBlock)S.getBlock(bi, bj);
         if (blk == null) {
            blk = new Matrix6dBlock();
            S.addBlock(bi, bj, blk);
         }
         blkNum = blk.getBlockNumber();
      }
      // nbr.setBlock (blk);
      nbr.setBlockNumber(blkNum);
   }
   
   
   /**
    * Add the velocity jacobian to a block of the global solve matrix. A block
    * corresponds to a node neighbor (node/nbr) of some node. The velocity 
    * jacobian is a property of the node neighbor itself.
    * 
    * Method is overridden to be compatible with 6x6 blocks.
    */
   @Override
   protected void addNeighborVelJacobian(
      SparseNumberedBlockMatrix M, FemNode3d node, NodeNeighbor nbr, double s) {
      if (nbr.getNode().getSolveIndex() != -1) {
         Matrix6dBlock blk =
         (Matrix6dBlock)M.getBlockByNumber(nbr.myBlkNum);
         if (nbr.getNode() == node && node.isActive()) {
            nbr.addVelJacobian(
               blk, s, myStiffnessDamping, myMassDamping);
         }
         else {
            nbr.addVelJacobian(blk, s, myStiffnessDamping, 0);
         }
      }
   }
}
