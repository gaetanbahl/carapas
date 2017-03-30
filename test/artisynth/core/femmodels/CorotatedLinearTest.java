package artisynth.core.femmodels;

import artisynth.core.materials.FemMaterial;
import artisynth.core.materials.LinearMaterial;
import artisynth.core.materials.SolidDeformation;
import maspack.matrix.Matrix3d;
import maspack.matrix.Matrix3dBase;
import maspack.matrix.Matrix3x3Block;
import maspack.matrix.Matrix6d;
import maspack.matrix.MatrixBlock;
import maspack.matrix.MatrixNd;
import maspack.matrix.RotationMatrix3d;
import maspack.matrix.SparseBlockMatrix;
import maspack.matrix.SymmetricMatrix3d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;

public class CorotatedLinearTest {


   /**
    * Compute stress and stiffness for warped linear material
    * @param mat
    * @param wR
    * @param e
    * @param D
    */
   public static void computeStressAndStiffness(FemMaterial mat, Matrix3d wR, FemElement3d e, Matrix6d D) {

      // clear everything
      for (FemNode3d node : e.getNodes()) {
         node.myInternalForce.setZero();
         for (FemNodeNeighbor nn : node.getNodeNeighbors()) {
            if (nn.myDivBlk != null) {
               nn.myDivBlk.setZero();
            }
            if (nn.myK != null) {
               nn.myK.setZero();
            }
            if (nn.myKX != null) {
               nn.myKX.setZero();
            }
         }
      }
      
      IntegrationPoint3d[] ipnts = e.getIntegrationPoints();
      IntegrationData3d[] idata = e.getIntegrationData();

      SymmetricMatrix3d linearStress = new SymmetricMatrix3d();
      SymmetricMatrix3d nonlinearStress = new SymmetricMatrix3d();
      Matrix6d linearD = null;
      Matrix6d nonlinearD = null;
      if (D != null) {
         D.setZero();
         linearD = new Matrix6d();
         nonlinearD = new Matrix6d();
      }

      e.setInverted(false); // will check this below

      SolidDeformation def = new SolidDeformation();
      FemIntegrationCoordinate mcoord = new FemIntegrationCoordinate();
      def.setMaterialCoordinate(mcoord);

      def.setR(wR);
      
      for (int k = 0; k < ipnts.length; k++) {
         IntegrationPoint3d pt = ipnts[k];
         IntegrationData3d dt = idata[k];

         mcoord.set(e, pt);
         pt.computeJacobianAndGradient(e.myNodes, idata[k].myInvJ0);

         //         if (mat.isWarping()) {
         //            // rotate F
         //            pt.F.mulTransposeLeft(def.getR(), pt.F);
         //         }
         def.setF(pt.F);

         double detJ = pt.computeInverseJacobian();

         // small-strain assumption for linear materials (J ~ I)
         // create new copy so GNx not overwritten
         double dv0 = idata[k].getDetJ0() * pt.getWeight();
         Vector3d[] GNx0 = pt.computeShapeGradient(idata[k].getInvJ0());
         
         double dv = detJ * pt.getWeight();
         Vector3d[] GNx = pt.computeShapeGradient(pt.myInvJ);

         // rotate shape initial function gradients
         if (wR != null) {
            for (Vector3d v : GNx0) {
               wR.mul(v);
            }
            
            // technically GNx would be unrotated by R^T first, then
            // rotated back later on when computing force/stiffness
         }

         
         Matrix3d Q = (dt.myFrame != null ? dt.myFrame : Matrix3d.IDENTITY);

         //         // rotate anisotropy
         //         if (mat.isWarping()) {
         //            Q.mulTransposeLeft(wR, Q);
         //         }

         linearStress.setZero();
         nonlinearStress.setZero();

         if (D != null) {
            linearD.setZero();
            nonlinearD.setZero();
         }

         if (mat.isLinear()) {
            mat.computeStress(linearStress, def, Q, null);
            if (D != null) {
               mat.computeTangent(linearD, linearStress, def, Q, null);
            }
         } else {
            mat.computeStress(nonlinearStress, def, Q, null);
            if (D != null) {
               mat.computeTangent(nonlinearD, nonlinearStress, def, Q, null);
            }
         }

         // add contribution to each node
         for (int i = 0; i < e.myNodes.length; i++) {
            FemNode3d nodei = e.myNodes[i];
            
            FemUtilities.addStressForce(
               nodei.myInternalForce, GNx0[i], linearStress, dv0);
            // FemUtilities.addStressForce(
            //   nodei.myInternalForce, GNx[i], nonlinearStress, dv);
            
            if (D != null) {
               D.scaledAdd(dv0, linearD);
               D.scaledAdd(dv, nonlinearD);

               for (int j = 0; j < e.myNodes.length; j++) {
                  e.myNbrs[i][j].addMaterialStiffness(GNx0[i], linearD, GNx0[j], dv0);
                  
                  // e.myNbrs[i][j].addMaterialStiffness(wR, GNx[i], nonlinearD, GNx[j], dv);
                  // e.myNbrs[i][j].addGeometricStiffness(GNx[i], nonlinearStress, GNx[j], dv);
               }
            }
         }
      }

   }
   
   // builds a Stiffness matrix, where entries are ordered by node numbers
   public static SparseBlockMatrix getStiffnessMatrix(FemElement3d elem) {

      SparseBlockMatrix M = new SparseBlockMatrix();
      int nnodes = elem.numNodes();
      int[] sizes = new int[nnodes];
      for (int i=0; i<nnodes; ++i) {
         sizes[i] = 3;
      }
      M.addRows (sizes, sizes.length);
      M.addCols (sizes, sizes.length);
      M.setVerticallyLinked (true);

      int idx = 0;
      for (FemNode3d node : elem.getNodes()) {
         node.setIndex(idx++);
      }

      // create solve blocks
      for (FemNode3d node : elem.getNodes()) {
         MatrixBlock blk = node.createSolveBlock();
         M.addBlock(node.getIndex(), node.getIndex(), blk);
      }
      for (int i = 0; i < elem.numNodes(); i++) {
         FemNode3d node = elem.getNodes()[i];
         for (FemNodeNeighbor nbr : node.getNodeNeighbors()) {
            FemNode3d other = nbr.getNode();
            Matrix3x3Block blk = null;
            if (other != node) {
               blk = new Matrix3x3Block();
               M.addBlock(node.getIndex(), nbr.getNode().getIndex(), blk);
            } else {
               blk = (Matrix3x3Block)M.getBlock(node.getIndex(), node.getIndex());
            }
            nbr.addPosJacobian(blk, -1.0);
         }
      }

      return M;
   }
   
   public static MatrixNd getForceVectors(FemElement3d elem) {
      
      int nnodes = elem.numNodes();
      MatrixNd F = new MatrixNd(3, nnodes);
      FemNode3d[] nodes = elem.getNodes();
      
      for (int i=0; i<nnodes; ++i) {
         Vector3d f = nodes[i].getInternalForce();
         F.setColumn(i, f);
      }
      
      return F;
   }
   
   public static VectorNd getDisplacementVector(FemElement3d elem) {
      int nnodes = elem.numNodes();
      VectorNd v = new VectorNd(3*nnodes);
      FemNode3d[] nodes = elem.getNodes();
      
      for (int i=0; i<nnodes; ++i) {
         Vector3d f = nodes[i].getDisplacement();
         v.set(3*i, f.x);
         v.set(3*i+1, f.y);
         v.set(3*i+2, f.z);
      }
      return v;
   }

   public void doRotationTest() {

      double[][] rest = {{0,0,0}, {1,0,0}, {0,1,0}, {0,0,1}};
      double[][] pos = {{0.2, 0.1, 0.1}, {1.3,0.4,0.2}, {-0.2, 1.4, 0.5}, {0.3,0.3, 0.9}};
      
      FemNode3d[] nodes = new FemNode3d[4];
      for (int i=0; i<4; ++i) {
         nodes[i] = new FemNode3d(rest[i][0], rest[i][1], rest[i][2]);
         nodes[i].setPosition(pos[i][0], pos[i][1], pos[i][2]);
      }
      
      TetElement tet = new TetElement(nodes);
      tet.connectToHierarchy();
      
      VectorNd u = getDisplacementVector(tet);
      
      LinearMaterial lmat = new LinearMaterial(150, 0.25, true);
      Matrix6d D = new Matrix6d();
      computeStressAndStiffness(lmat, Matrix3d.IDENTITY, tet, D);
      
      SparseBlockMatrix K = getStiffnessMatrix(tet);
      System.out.println("Stiffness: ");
      System.out.println(K.toString("%.2f").replaceAll("-0.00", "0.00"));
      
      MatrixNd F = getForceVectors(tet);
      System.out.println("Forces: ");
      System.out.println(F.toString("%.2f").replaceAll("-0.00", "0.00"));
      
      VectorNd Ku = new VectorNd();
      Ku.mul(K, u);
      double uKu = Ku.dot(u);
      System.out.println("Ku: " + Ku.toString("%.2f").replaceAll("-0.00", "0.00"));
      System.out.println("Stiffness energy: " + uKu);
      
      // rotate everything by 90 degrees and check forces
      RotationMatrix3d R = new RotationMatrix3d();
      R.set(RotationMatrix3d.ROT_X_90);
      RotationMatrix3d Rinv = new RotationMatrix3d(R);
      Rinv.transpose();
      
      // transform all nodes
      for (FemNode3d node : nodes) {
         node.getPosition().transform(R);
      }
      computeStressAndStiffness(lmat, new Matrix3d(R), tet, D);
      SparseBlockMatrix K2 = getStiffnessMatrix(tet);
      System.out.println("Rotated Stiffness: ");
      System.out.println(K2.toString("%.2f").replaceAll("-0.00", "0.00"));
      
      MatrixNd F2 = getForceVectors(tet);
      System.out.println("Rotated Forces: ");
      System.out.println(F2.toString("%.2f").replaceAll("-0.00", "0.00"));
      
      VectorNd u2 = new VectorNd(u);
      rotateDisplacement(u2, R);
      VectorNd Ku2 = new VectorNd();
      Ku2.mul(K2, u2);
      double uKu2 = Ku2.dot(u2);
      
      System.out.println("Ku: " + Ku2.toString("%.2f").replaceAll("-0.00", "0.00"));
      System.out.println("Stiffness energy: " + uKu2);
      
      // rotate stiffness
      rotateStiffness(K2, Rinv);
      System.out.println("Rotated rotated Stiffness: ");
      System.out.println(K2.toString("%.2f").replaceAll("-0.00", "0.00"));
   }
   
   public static void rotateDisplacement(VectorNd u, RotationMatrix3d R) {
      int nx = u.size()/3;
      for (int i=0; i<nx; ++i) {
         Vector3d uu = new Vector3d(u.get(3*i), u.get(3*i+1), u.get(3*i+2));
         R.mul(uu);
         u.setSubVector(3*i, uu);
      }
   }
   
   public static void rotateStiffness(SparseBlockMatrix K, Matrix3dBase R) {
      for (int i=0; i<K.numBlockRows(); ++i) {
         for (int j=0; j<K.numBlockCols(); ++j) {
            Matrix3x3Block block = (Matrix3x3Block)K.getBlock(i, j);
            block.mulTranspose(R);
            block.mul(R, block);
         }
      }
   }
   
   public static void main(String[] args) {
      CorotatedLinearTest tester = new CorotatedLinearTest();
      tester.doRotationTest();
      
   }

}
