package artisynth.core.femmodels;

import artisynth.core.femmodels.FemModel.IncompMethod;
import artisynth.core.materials.FemMaterial;
import artisynth.core.materials.IncompressibleMaterial;
import artisynth.core.materials.LinearMaterial;
import artisynth.core.materials.SolidDeformation;
import artisynth.core.materials.ViscoelasticBehavior;
import artisynth.core.materials.ViscoelasticState;
import artisynth.core.mechmodels.PointList;
import maspack.matrix.Matrix3d;
import maspack.matrix.Matrix6d;
import maspack.matrix.MatrixBlock;
import maspack.matrix.SymmetricMatrix3d;
import maspack.matrix.Vector3d;

public class ShellFemModel3d extends FemModel3d {
   
   public ShellFemModel3d () {
      this(null);
   }
   
   public ShellFemModel3d (String name) {
      super(name);
   }
   
   /**
    * Compute the director vector for each node of this model. Should be invoked 
    * after the nodes and elements are added to this model.
    * 
    * FEBio port notes:
    *   FEMesh::InitShellsNew().
    *   
    * Ordering of element's node matters
    * 
    */
   public void initNodeDirectors() {
      /* Absolute and relative position of nodes probably doesn't 
       * matter b/c vector sub and normalization is being used. */
      for (FemElement3d ele : this.myElements) {
         ShellFemElement3d sEle = (ShellFemElement3d) ele;
         for (int i = 0; i < sEle.numNodes(); i++) {     
            // Get next and prev nodes relative to i-th node.
            int n = (i+1) % sEle.numNodes();
            int p = (i==0) ? sEle.numNodes()-1 : i-1; 
            
            Vector3d iPos = sEle.myNodes[i].getRestPosition ();
            Vector3d nPos = sEle.myNodes[n].getRestPosition ();
            Vector3d pPos = sEle.myNodes[p].getRestPosition ();
            
            Vector3d n_i = new Vector3d();
            n_i.sub (nPos, iPos);
            
            Vector3d p_i = new Vector3d();
            p_i.sub (pPos, iPos);
            
            Vector3d dir = new Vector3d();
            dir.cross (n_i, p_i);
            dir.normalize ();
            dir.scale (sEle.myShellThickness);
            
            if (sEle.myNodes[i].myDirector0 == null) {
               sEle.myNodes[i].myDirector0 = dir;
            }
            else {
               sEle.myNodes[i].myDirector0.add(dir);
            }
         }
      }
      
      // Average the directors.
      for (FemNode3d node : myNodes) {
         int numAdjFaces = node.numAdjacentElements ();
         System.out.println ("node index: " + node.getIndex ());
         System.out.println ("numAdjFaces: " + numAdjFaces);
         node.myDirector0.scale(1/numAdjFaces);
      }
   }
   
   
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
         wpnt.computeJacobianAndGradient(e);       
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
         if (e instanceof ShellQuadTetElement) {
            ((ShellQuadTetElement)e).getAreaWeightedNormals(myNodalConstraints);
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
            pt.computeJacobianAndGradient(e);           // DANNY HERE
            def.setF(pt.F);
            double detJ = pt.computeInverseJacobian();
            if (detJ < myMinDetJ) {
               myMinDetJ = detJ;
               myMinDetJElement = e;
            }
            // SKIPPED
            if (detJ <= 0 && !e.materialsAreInvertible()) {
               e.setInverted(true);
               // TODO DANNY: problem here
               myNumInverted++;
            }
            double dv = detJ * pt.getWeight();
            Vector3d[] GNx = pt.updateShapeGradient(pt.myInvJ);

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
               if (e instanceof ShellQuadTetElement) {
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
               FemNode3d nodei = e.myNodes[i];
               int bi = nodei.getSolveIndex();
               
               // Add stress (pt.sigma) to node force
               FemUtilities.addShellStressForce(
                  nodei.myInternalForce, pt.sigma, dv, i, pt, e);

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
                        for (FemNodeNeighbor nbr : getNodeNeighbors(nodei)) {
                           int j = e.getLocalNodeIndex(nbr.myNode);
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
                           
                           double iN = e.getN(i, pt.coords);
                           double jN = e.getN(j, pt.coords);
                           
                           Vector3d idN = new Vector3d();
                           e.getdNds(idN, i, pt.coords);
                           
                           Vector3d jdN = new Vector3d();
                           e.getdNds(jdN, j, pt.coords);
                           
                           double t = pt.coords.z; 
                           
                           Vector3d[] gct = pt.getContraBaseVectors(e);
                           
                           e.myNbrs[i][j].addShellMaterialStiffness (
                              iN, jN, idN, jdN, dv, t, gct, 
                              /*material stress=*/ pt.sigma, 
                              /*material tangent=*/ D,
                              GNx[i], GNx[j], p);

                           // NEW
                           
                           
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
               for (FemNodeNeighbor nbr : getNodeNeighbors(e.myNodes[k])) {
                  int j = e.getLocalNodeIndex(nbr.myNode);
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

   
   @Override 
   protected void computeAvgGNx(FemElement3d ele) {
      ShellFemElement3d e = (ShellFemElement3d)ele;
      
      ShellIntegrationPoint3d[] ipnts = e.getIntegrationPoints();
      ShellIntegrationData3d[] idata = e.getIntegrationData();

      Vector3d[] avgGNx = null;
      MatrixBlock[] constraints = null;

      constraints = e.getIncompressConstraints();
      for (int i = 0; i < e.myNodes.length; i++) {
         constraints[i].setZero();
      }

      e.setInverted(false);
      for (int k = 0; k < ipnts.length; k++) {
         ShellIntegrationPoint3d pt = ipnts[k];
         pt.computeJacobianAndGradient(e);
         double detJ = pt.computeInverseJacobian();
         if (detJ <= 0) {
            e.setInverted(true);
            // if (abortOnInvertedElems) {
            // throw new NumericalException ("Inverted elements");
            // }
         }
         double dv = detJ * pt.getWeight();
         Vector3d[] GNx = pt.updateShapeGradient(pt.myInvJ);

         double[] H = pt.getPressureWeights().getBuffer();
         for (int i = 0; i < e.myNodes.length; i++) {
            FemUtilities.addToIncompressConstraints(
               constraints[i], H, GNx[i], dv);
         }
      }
   }

}
