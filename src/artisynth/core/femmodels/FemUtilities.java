/**
 * Copyright (c) 2014, by the Authors: John E Lloyd (UBC)
 *
 * This software is freely available under a 2-clause BSD license. Please see
 * the LICENSE file in the ArtiSynth distribution directory for details.
 */
package artisynth.core.femmodels;

import javax.swing.plaf.synth.SynthSplitPaneUI;

import artisynth.core.materials.TensorUtils;
import maspack.matrix.*;

/** 
 * Provides some general utilities for FEM computations. Some of these
 * involve the B matrix formed from the shape function gradient dNdx.
 * If this gradient is represented as g, and if the stress tensor
 * is represented with the vector
 * <pre>
 *   sigma = [ sxx syy szz sxy syz sxz ]
 * </pre>
 * then the B matrix takes the form
 * <pre>
 *      [ gx  0  0 ]           00 11 22 01 12 02
 *      [  0 gy  0 ]
 *      [  0  0 gz ]     T   [ gx  0  0 gy  0 gz ]
 *  B = [ gy gx  0 ]    B  = [  0 gy  0 gx gz  0 ]
 *      [  0 gz gy ]         [  0  0 gz  0 gy gx ]
 *      [ gz  0 gx ]
 * </pre>
 */
public class FemUtilities {
   
   /** 
    * Adds the force on a node resulting from a given stress and shape
    * function gradient g. This is done by computing f = B^T sig, where B is
    * the matrix formed from the shape function gradient.
    */
   public static void addStressForce (
      Vector3d f, Vector3d g, SymmetricMatrix3d sig, double dv) {
      double gx = g.x*dv;
      double gy = g.y*dv;
      double gz = g.z*dv;

      f.x += gx*sig.m00 + gy*sig.m01 + gz*sig.m02;
      f.y += gy*sig.m11 + gx*sig.m01 + gz*sig.m12;
      f.z += gz*sig.m22 + gy*sig.m12 + gx*sig.m02;
   }

   /** 
    * Adds a weighted node-to-node stiffness to the matrix Kij via the formula
    * <pre>
    *  Kij = (Bi^T D Bj + (gi^T sig gj) I)*dv
    * </pre>
    * where gi and gj are the gradients for shape functions i and j,
    * Bi and Bj are the B matrices formed from gi and gj, 
    * D is the derivative of the constitutive relationship, and
    * dv is the weighting term.
    */
   public static void addMaterialStiffness (
      Matrix3d K, Vector3d gi, Matrix6d D,
      SymmetricMatrix3d sig, Vector3d gj, double dv) {
      addMaterialStiffness(K, gi, D, gj, dv);
      addGeometricStiffness(K, gi, sig, gj, dv);
   }
   
   /** 
    * Adds a weighted node-to-node stiffness to the matrix Kij via the formula
    * <pre>
    *  Kij = Bi^T D Bj dv
    * </pre>
    * where gi and gj are the gradients for shape functions i and j,
    * Bi and Bj are the B matrices formed from gi and gj, 
    * D is the derivative of the constitutive relationship, and
    * dv is the weighting term.
    */
   public static void addMaterialStiffness (
      Matrix3d K, Vector3d gi, Matrix6d D, Vector3d gj, double dv) {
      double gjx = gj.x*dv;
      double gjy = gj.y*dv;
      double gjz = gj.z*dv;

      double dm00 = D.m00*gjx + D.m03*gjy + D.m05*gjz;
      double dm01 = D.m01*gjy + D.m03*gjx + D.m04*gjz;
      double dm02 = D.m02*gjz + D.m04*gjy + D.m05*gjx;
      
      double dm10 = D.m10*gjx + D.m13*gjy + D.m15*gjz;
      double dm11 = D.m11*gjy + D.m13*gjx + D.m14*gjz;
      double dm12 = D.m12*gjz + D.m14*gjy + D.m15*gjx;
      
      double dm20 = D.m20*gjx + D.m23*gjy + D.m25*gjz;
      double dm21 = D.m21*gjy + D.m23*gjx + D.m24*gjz;
      double dm22 = D.m22*gjz + D.m24*gjy + D.m25*gjx;
      
      double dm30 = D.m30*gjx + D.m33*gjy + D.m35*gjz;
      double dm31 = D.m31*gjy + D.m33*gjx + D.m34*gjz;
      double dm32 = D.m32*gjz + D.m34*gjy + D.m35*gjx;
      
      double dm40 = D.m40*gjx + D.m43*gjy + D.m45*gjz;
      double dm41 = D.m41*gjy + D.m43*gjx + D.m44*gjz;
      double dm42 = D.m42*gjz + D.m44*gjy + D.m45*gjx;
      
      double dm50 = D.m50*gjx + D.m53*gjy + D.m55*gjz;
      double dm51 = D.m51*gjy + D.m53*gjx + D.m54*gjz;
      double dm52 = D.m52*gjz + D.m54*gjy + D.m55*gjx;

      double gix = gi.x;
      double giy = gi.y;
      double giz = gi.z;

      K.m00 += gix*dm00 + giy*dm30 + giz*dm50;
      K.m01 += gix*dm01 + giy*dm31 + giz*dm51;
      K.m02 += gix*dm02 + giy*dm32 + giz*dm52;

      K.m10 += giy*dm10 + gix*dm30 + giz*dm40;
      K.m11 += giy*dm11 + gix*dm31 + giz*dm41;
      K.m12 += giy*dm12 + gix*dm32 + giz*dm42;

      K.m20 += giz*dm20 + giy*dm40 + gix*dm50;
      K.m21 += giz*dm21 + giy*dm41 + gix*dm51;
      K.m22 += giz*dm22 + giy*dm42 + gix*dm52;
   }
   
   /**
    * Adds the geometric stiffness defined by
    * <pre> 
    * Kij += (gi^T sig gj) I*dv
    * </pre>
    */
   public static void addGeometricStiffness(Matrix3d K, Vector3d gi, 
      SymmetricMatrix3d sig, Vector3d gj, double dv) {

      // add geometrical stiffness
      double Kg = (
         gi.x*(sig.m00*gj.x + sig.m01*gj.y + sig.m02*gj.z) +
         gi.y*(sig.m10*gj.x + sig.m11*gj.y + sig.m12*gj.z) +
         gi.z*(sig.m20*gj.x + sig.m21*gj.y + sig.m22*gj.z));
      Kg = Kg*dv;

      K.m00 += Kg;
      K.m11 += Kg;
      K.m22 += Kg;
   }

   /** 
    * Adds a weighted node-to-node stiffness to the matrix Kij via the formula
    * <pre>
    *  Kij = Bi^T D Bj dv
    * </pre>
    * where gi and gj are the gradients for shape functions i and j,
    * Bi and Bj are the B matrices formed from gi and gj,
    * D is the linear stiffness relationship associated with
    * Youngs modulus E and Poissons ratio nu, and dv is the weighting term.
    */
   public static void addMaterialStiffness (
      Matrix3d K, Vector3d gi, double E, double nu,
      Vector3d gj, double dv) {

      double s = E/(1+nu);
      double dia = s*(1-nu)/(1-2*nu);
      double off = s*nu/(1-2*nu);
      double di2 = 0.5*s;
      
      double gjx = gj.x*dv;
      double gjy = gj.y*dv;
      double gjz = gj.z*dv;
      
      double dgjx = dia*gjx;
      double dgjy = dia*gjy;
      double dgjz = dia*gjz;
      
      double ogjx = off*gjx;
      double ogjy = off*gjy;
      double ogjz = off*gjz;
      
      double d2gjx = di2*gjx;
      double d2gjy = di2*gjy;
      double d2gjz = di2*gjz;

      double gix = gi.x;
      double giy = gi.y;
      double giz = gi.z;

      double gixd2gjx = gix*d2gjx;
      double giyd2gjy = giy*d2gjy;
      double gizd2gjz = giz*d2gjz;
      
      K.m00 += gix*dgjx + giyd2gjy + gizd2gjz;
      K.m01 += gix*ogjy + giy*d2gjx;
      K.m02 += gix*ogjz + giz*d2gjx;

      K.m10 += giy*ogjx + gix*d2gjy;
      K.m11 += giy*dgjy + gixd2gjx + gizd2gjz;
      K.m12 += giy*ogjz + giz*d2gjy;

      K.m20 += giz*ogjx + gix*d2gjz;
      K.m21 += giz*ogjy + giy*d2gjz;
      K.m22 += giz*dgjz + giyd2gjy + gixd2gjx;

   }
   
   /** 
    * Adds pressure stiffness to the matrix Kij via the formula
    * <pre>
    *  Kij = Bi^T D Bj
    * </pre>
    * where gi and gj are the gradients for shape functions i and j,
    * Bi and Bj are the B matrices formed from gi and gj, and
    * D is created from the pressure p according to D_ii = -p
    * and D_ij = p for i != j and i, j &lt; 3.
    */
   public static void addPressureStiffness (
      Matrix3d K, Vector3d gi, double p,
      Vector3d gj, double dv) {

      double gjx = p*gj.x*dv;
      double gjy = p*gj.y*dv;
      double gjz = p*gj.z*dv;

      double gix = gi.x;
      double giy = gi.y;
      double giz = gi.z;

      double diag = -gix*gjx - giy*gjy - giz*gjz;

      K.m00 += diag;
      K.m01 += gix*gjy - giy*gjx;
      K.m02 += gix*gjz - giz*gjx;

      K.m10 += giy*gjx - gix*gjy;
      K.m11 += diag;
      K.m12 += giy*gjz - giz*gjy;

      K.m20 += giz*gjx - gix*gjz;
      K.m21 += giz*gjy - giy*gjz;
      K.m22 += diag;

   }

   /** 
    * Adds dilational stiffness to the node-to-node stiffness matrix Kij.
    * This is calculated via the formula
    * <pre>
    *  Kij += kp (intGi intGj^T)
    * </pre>
    * where intGi and intGj are the integrals (over the element)
    * of the gradients for shape functions i and j, and kp is (typically)
    * the bulkModulus divided by the restVolume.
    */
   public static void addDilationalStiffness (
      Matrix3d K, double kp, Vector3d intGi, Vector3d intGj) {

      double gix = kp*intGi.x;
      double giy = kp*intGi.y;
      double giz = kp*intGi.z;    

      double gjx = intGj.x;
      double gjy = intGj.y;
      double gjz = intGj.z;

      K.m00 += gix*gjx;
      K.m01 += gix*gjy;
      K.m02 += gix*gjz;

      K.m10 += giy*gjx;
      K.m11 += giy*gjy;
      K.m12 += giy*gjz;

      K.m20 += giz*gjx;
      K.m21 += giz*gjy;
      K.m22 += giz*gjz;
   }

   /** 
    * Adds dilational stiffness to the node-to-node stiffness matrix Kij.
    * This is calculated via the formula
    * <pre>
    *  Kij += GT_i Kp GT_j^T
    * </pre>
    * where GT_i and GT_j are the incompressibility constraints for nodes
    * i and j and Kp is a diagonal matrix of stiffness pressures.
    */
   public static void addDilationalStiffness (
      Matrix3d K, double[] Kp, MatrixBlock GT_i, MatrixBlock GT_j) {

      if (GT_i.colSize() == 1) {
         Matrix3x1.mulScaledTransposeRightAdd (
            K, Kp[0], (Matrix3x1Block)GT_i, (Matrix3x1Block)GT_j);
      }
      else if (GT_i.colSize() == 2) {
         Matrix3x2.mulScaledTransposeRightAdd (
            K, (Matrix3x2Block)GT_i, Kp, (Matrix3x2Block)GT_j);
      }
      else if (GT_i.colSize() == 4) {
         Matrix3x4.mulScaledTransposeRightAdd (
            K, (Matrix3x4Block)GT_i, Kp, (Matrix3x4Block)GT_j);
      }
      else {
         MatrixNd G_j = new MatrixNd (GT_j.colSize(), 3);
         MatrixNd tmp = new MatrixNd (3,3);
         G_j.transpose ((MatrixNdBlock)GT_j);
         G_j.mulDiagonalLeft (Kp);
         tmp.mul ((MatrixNdBlock)GT_i, G_j);
         K.m00 += tmp.get(0,0);
         K.m01 += tmp.get(0,1);
         K.m02 += tmp.get(0,2);
         K.m10 += tmp.get(1,0);
         K.m11 += tmp.get(1,1);
         K.m12 += tmp.get(1,2);
         K.m20 += tmp.get(2,0);
         K.m21 += tmp.get(2,1);
         K.m22 += tmp.get(2,2);
      }
      // else {
      //    throw new InternalErrorException (
      //       "Support not present for "+GT_i.colSize()+" pressure DOFs");
      // }
      
   }

   /** 
    * Adds dilational stiffness to the node-to-node stiffness matrix Kij.
    * This is calculated via the formula
    * <pre>
    *  Kij += GT_i Rinv GT_j^T
    * </pre>
    * where GT_i and GT_j are the incompressibility constraints for nodes
    * i and j and Kp is a diagonal matrix of stiffness pressures.
    */
   public static void addDilationalStiffness (
      Matrix3d K, MatrixNd Rinv, MatrixBlock GT_i, MatrixBlock GT_j) {

      if (GT_i.colSize() == 1) {
         Matrix3x1.mulScaledTransposeRightAdd (
            K, Rinv.get(0,0), (Matrix3x1Block)GT_i, (Matrix3x1Block)GT_j);
      }
      else {
         MatrixNd X = new MatrixNd (GT_i.colSize(), 3);
         MatrixNd GTj = new MatrixNd (GT_j);
         MatrixNd GTi = new MatrixNd (GT_i);
         MatrixNd Kij = new MatrixNd (3, 3);
         X.mulTransposeRight (Rinv, GTj);
         Kij.mul (GTi, X);

         K.m00 += Kij.get(0,0);
         K.m01 += Kij.get(0,1);
         K.m02 += Kij.get(0,2);
         K.m10 += Kij.get(1,0);
         K.m11 += Kij.get(1,1);
         K.m12 += Kij.get(1,2);
         K.m20 += Kij.get(2,0);
         K.m21 += Kij.get(2,1);
         K.m22 += Kij.get(2,2);
      }
   }

   /** 
    * Adds stiffness for the incompressibility constraint to the node-to-node
    * stiffness matrix Kij.  This is calculated via the formula
    * 
    * <pre>
    *  Kij += kp (intGi intGj^T - intGj intGi^T)
    * </pre>
    * where intGi and intGj are the integrals (over the element)
    * of the gradients for shape functions i and j, and s is (typically)
    * the determinant of F times the current pressure.
    */
   public static void addIncompressibilityStiffness (
      Matrix3d K, double s, Vector3d intGi, Vector3d intGj) {

      double gix = s*intGi.x;
      double giy = s*intGi.y;
      double giz = s*intGi.z;    

      double gjx = intGj.x;
      double gjy = intGj.y;
      double gjz = intGj.z;    

      //K.m00 += (gix*gjx - gjx*gix); don't need to compute; is zero
      K.m01 += (gix*gjy - gjx*giy);
      K.m02 += (gix*gjz - gjx*giz);

      K.m10 += (giy*gjx - gjy*gix);
      //K.m11 += (giy*gjy - gjy*giy); don't need to compute; is zero
      K.m12 += (giy*gjz - gjy*giz);

      K.m20 += (giz*gjx - gjz*gix);
      K.m21 += (giz*gjy - gjz*giy);
      //K.m22 += (giz*gjz - gjz*giz); don't need to compute; is zero
   }

   /** 
    * Adds H^T GNx dv to a matrix block, where H is a row vector of weight
    * values, and dv is a volume differential. It is assumed that the matrix
    * block has a row size of 3.
    */
   public static void addToIncompressConstraints (
      MatrixBlock blk, double[] H, Vector3d GNx, double dv) {

      double h;
      if (blk.colSize() == 1) {
         Matrix3x1Block M = (Matrix3x1Block)blk;
         h = H[0]*dv;
         M.m00 += h*GNx.x;
         M.m10 += h*GNx.y;
         M.m20 += h*GNx.z;
      }
      else if (blk.colSize() == 2) {
         Matrix3x2Block M = (Matrix3x2Block)blk;
         h = H[0]*dv;
         M.m00 += h*GNx.x;
         M.m10 += h*GNx.y;
         M.m20 += h*GNx.z;

         h = H[1]*dv;
         M.m01 += h*GNx.x;
         M.m11 += h*GNx.y;
         M.m21 += h*GNx.z;
      }
      else if (blk.colSize() == 4) {
         Matrix3x4Block M = (Matrix3x4Block)blk;
         h = H[0]*dv;
         M.m00 += h*GNx.x;
         M.m10 += h*GNx.y;
         M.m20 += h*GNx.z;

         h = H[1]*dv;
         M.m01 += h*GNx.x;
         M.m11 += h*GNx.y;
         M.m21 += h*GNx.z;

         h = H[2]*dv;
         M.m02 += h*GNx.x;
         M.m12 += h*GNx.y;
         M.m22 += h*GNx.z;

         h = H[3]*dv;
         M.m03 += h*GNx.x;
         M.m13 += h*GNx.y;
         M.m23 += h*GNx.z;
      }
      else {
         MatrixNdBlock M = (MatrixNdBlock)blk;
         double[] buf = M.getBuffer();
         int nc = blk.colSize();
         for (int k=0; k<blk.colSize(); k++) {
            h = H[k]*dv;
            buf[0*nc+k] += h*GNx.x;
            buf[1*nc+k] += h*GNx.y;
            buf[2*nc+k] += h*GNx.z;
         }
      }
      // else {
      //    System.out.println ("M=" + blk.getClass());
      //    throw new InternalErrorException (
      //       "Support not present for "+blk.colSize()+" pressure DOFs");
      // }
   }

   private static void setTriangle (
      FemNode3d[] tri, FemNode3d n0, FemNode3d n1, FemNode3d n2) {
      tri[0] = n0;
      tri[1] = n1;
      tri[2] = n2;
   }   

   public static FemNode3d[][] triangulate8NodeFace (FaceNodes3d face) {
      FemNode3d[] nodes = face.getNodes();
      if (nodes.length != 8) {
         throw new IllegalArgumentException (
            "Expecting 8 nodes, got " + nodes.length);
      }
      FemNode3d[][] triangles = new FemNode3d[6][3];

      setTriangle (triangles[0], nodes[0], nodes[1], nodes[7]);
      setTriangle (triangles[1], nodes[1], nodes[2], nodes[3]);
      setTriangle (triangles[2], nodes[3], nodes[4], nodes[5]);
      setTriangle (triangles[3], nodes[7], nodes[5], nodes[6]);
      if (nodes[7].distance(nodes[3]) < nodes[1].distance(nodes[5])) {
         setTriangle (triangles[4], nodes[1], nodes[3], nodes[7]);
         setTriangle (triangles[5], nodes[7], nodes[3], nodes[5]);
      }
      else {
         setTriangle (triangles[4], nodes[1], nodes[5], nodes[7]);
         setTriangle (triangles[5], nodes[1], nodes[3], nodes[5]);
      }
      return triangles;
   }  

   public static FemNode3d[][] triangulate6NodeFace (FaceNodes3d face) {
      FemNode3d[] nodes = face.getNodes();
      if (nodes.length != 6) {
         throw new IllegalArgumentException (
            "Expecting 6 nodes, got " + nodes.length);
      }
      FemNode3d[][] triangles = new FemNode3d[4][3];

      if (nodes[1].distance(nodes[4]) < nodes[5].distance(nodes[3])) {
         setTriangle (triangles[0], nodes[0], nodes[1], nodes[5]);
         setTriangle (triangles[1], nodes[1], nodes[4], nodes[5]);
         setTriangle (triangles[2], nodes[1], nodes[3], nodes[4]);
         setTriangle (triangles[3], nodes[1], nodes[2], nodes[3]);
      }
      else {
         setTriangle (triangles[0], nodes[0], nodes[1], nodes[5]);
         setTriangle (triangles[1], nodes[1], nodes[3], nodes[5]);
         setTriangle (triangles[2], nodes[3], nodes[4], nodes[5]);
         setTriangle (triangles[3], nodes[1], nodes[2], nodes[3]);
      }
      return triangles;
   }

   private static int setTriIndices (
      int[] indices, int idx, int i0, int i1, int i2) {
      indices[idx++] = i0;
      indices[idx++] = i1;
      indices[idx++] = i2;
      return idx;
   }

   /** 
    * Helper method to triangulate a set of faces. Each face is given by a set
    * of nodes, arranged counter-clockwise about what would be its outer facing
    * normal if all the nodes were coplanar. Information for all
    * faces is contained in <code>faces</code>, with each entry
    * consisting of the number of nodes, followed by the indices for
    * each node.
    */
   static int[] triangulateFaceIndices (int[] faces) {
      
      int numtri = 0;

      int k = 0;
      while (k < faces.length) {
         int n = faces[k++];
         switch (n) {
            case 3: numtri += 1; break;
            case 4: numtri += 2; break;
            case 6: numtri += 4; break;
            case 8: numtri += 6; break;
            default:
               throw new UnsupportedOperationException (
                  "Triangulation of faces with "+faces[k]+" unsupported");
         }
         k += n;
      }
      int[] trif = new int[3*numtri];
      //System.out.println ("trif.length = " + 3*numtri);

      k = 0;
      int i = 0;
      while (k < faces.length) {
         int n = faces[k++];
         switch (n) {
            case 3: {
               i = setTriIndices (trif, i, faces[k], faces[k+1], faces[k+2]);
               break;
            }
            case 4: {
               i = setTriIndices (trif, i, faces[k], faces[k+1], faces[k+2]);
               i = setTriIndices (trif, i, faces[k], faces[k+2], faces[k+3]);
               break;
            }
            case 6: {
               i = setTriIndices (trif, i, faces[k], faces[k+1], faces[k+5]);
               i = setTriIndices (trif, i, faces[k+1], faces[k+3], faces[k+5]);
               i = setTriIndices (trif, i, faces[k+1], faces[k+2], faces[k+3]);
               i = setTriIndices (trif, i, faces[k+5], faces[k+3], faces[k+4]);
               break;
            }
            case 8: {
               i = setTriIndices (trif, i, faces[k], faces[k+1], faces[k+7]);
               i = setTriIndices (trif, i, faces[k+1], faces[k+5], faces[k+7]);
               i = setTriIndices (trif, i, faces[k+1], faces[k+3], faces[k+5]);
               i = setTriIndices (trif, i, faces[k+1], faces[k+2], faces[k+3]);
               i = setTriIndices (trif, i, faces[k+3], faces[k+4], faces[k+5]);
               i = setTriIndices (trif, i, faces[k+7], faces[k+5], faces[k+6]);
               break;
            }
         }
         k += n;
      }
      return trif;
   }

   
   /* --- SHELL-SPECIFIC METHODS --- */
   
   
   /** 
    * Add weighted material stiffness for this i,j node neighbor pair
    * (represented by 3x3 stiffness block), relative to a particular integration
    * point of the shell element.
    * 
    * @param K
    * 3x3 stiffness block, belonging to i-j node pair, to be increased with 
    * material stiffness.
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
    * @postcond
    * K is increased.
    * 
    * FEBio: FEElasticShellDomain::ElementStiffness
    *   
    *   TODO: Shell-element in FEBio uses 6 dof (3 for x,y,z displacement, 
    *         and 3 for x,y,z rotation). Artisynth is only hard coded for 
    *         3 dof, so individual stiffness block of i,j node pair is only 
    *         3x3 rather than 6x6. This function was adjusted for 3dof only.
    *         Not sure if the full 6dof is needed.
    */
   public static void addShellMaterialStiffness (
      Matrix6d K, double iN, double jN, Vector3d idN, Vector3d jdN, double dv,
      double t, Vector3d[] gct, SymmetricMatrix3d matStress,
      Matrix6d matTangent) {
     
      // Compute gradM for i and j
      
      Vector3d iGradM = new Vector3d();
      iGradM.scaledAdd(idN.x, gct[0]);
      iGradM.scaledAdd(idN.y, gct[1]);
      
      Vector3d jGradM = new Vector3d();
      jGradM.scaledAdd(jdN.x, gct[0]);
      jGradM.scaledAdd(jdN.y, gct[1]);
      
      // Compute gradMu for i and j
      
      Vector3d iGradMu = new Vector3d();
      iGradMu.scaledAdd(1+t, iGradM);
      iGradMu.scaledAdd(iN, gct[2]);
      iGradMu.scale(0.5);
      
      Vector3d jGradMu = new Vector3d();
      jGradMu.scaledAdd(1+t, jGradM);
      jGradMu.scaledAdd(jN, gct[2]);
      jGradMu.scale(0.5);
      
      // Compute gradMd for i and j
      
      Vector3d iGradMd = new Vector3d();
      iGradMd.scaledAdd (1-t, iGradM);
      iGradMd.scaledAdd (-iN, gct[2]);
      iGradMd.scale (0.5);
      
      Vector3d jGradMd = new Vector3d();
      jGradMd.scaledAdd (1-t, jGradM);
      jGradMd.scaledAdd (-jN, gct[2]);
      jGradMd.scale (0.5);
      
      Matrix3d Kuu = new Matrix3d();
      TensorUtils.v3DotTens4sDotv3 (/*out=*/Kuu, iGradMu, matTangent, jGradMu);
      Kuu.scale(dv);
      
      Matrix3d Kud = new Matrix3d();
      TensorUtils.v3DotTens4sDotv3 (Kud, iGradMu, matTangent, jGradMd);
      Kud.scale (dv);
      
      Matrix3d Kdu = new Matrix3d();
      TensorUtils.v3DotTens4sDotv3 (Kdu, iGradMd, matTangent, jGradMu);
      Kdu.scale (dv);
      
      Matrix3d Kdd = new Matrix3d();
      TensorUtils.v3DotTens4sDotv3 (Kdd, iGradMd, matTangent, jGradMd);
      Kdd.scale (dv);
      
      // Material component 
      
      K.m00 += Kuu.m00;  K.m01 += Kuu.m01;  K.m02 += Kuu.m02;
      K.m10 += Kuu.m10;  K.m11 += Kuu.m11;  K.m12 += Kuu.m12;
      K.m20 += Kuu.m20;  K.m21 += Kuu.m21;  K.m22 += Kuu.m22;
      
      K.m03 += Kud.m00;  K.m04 += Kud.m01;  K.m05 += Kud.m02;
      K.m13 += Kud.m10;  K.m14 += Kud.m11;  K.m15 += Kud.m12;
      K.m23 += Kud.m20;  K.m24 += Kud.m21;  K.m25 += Kud.m22;
      
      K.m30 += Kdu.m00;  K.m31 += Kdu.m01;  K.m32 += Kdu.m02;
      K.m40 += Kdu.m10;  K.m41 += Kdu.m11;  K.m42 += Kdu.m12;
      K.m50 += Kdu.m20;  K.m51 += Kdu.m21;  K.m52 += Kdu.m22;
      
      K.m33 += Kdd.m00;  K.m34 += Kdd.m01;  K.m35 += Kdd.m02;
      K.m43 += Kdd.m10;  K.m44 += Kdd.m11;  K.m45 += Kdd.m12;
      K.m53 += Kdd.m20;  K.m54 += Kdd.m21;  K.m55 += Kdd.m22;
      
      // Stress component 
      
      Vector3d sjGradMu = new Vector3d( jGradMu );   
      matStress.mul (sjGradMu);
      
      Vector3d sjGradMd = new Vector3d( jGradMd ); 
      matStress.mul (sjGradMd);
                              
      double sKuu = iGradMu.dot(sjGradMu) * dv;            
      double sKud = iGradMu.dot(sjGradMd) * dv;
      double sKdu = iGradMd.dot(sjGradMu) * dv;
      double sKdd = iGradMd.dot(sjGradMd) * dv;
      
      K.m00 += sKuu;
      K.m11 += sKuu;
      K.m22 += sKuu; 
      
      K.m03 += sKud;
      K.m14 += sKud;
      K.m25 += sKud; 
      
      K.m30 += sKdu;
      K.m41 += sKdu;
      K.m52 += sKdu;
      
      K.m33 += sKdd;
      K.m44 += sKdd;
      K.m55 += sKdd;
   }
   
   /** 
    * Adds the force on a node resulting from a given stress at a given 
    * integration point. This only applies to shell elements.
    * 
    * @param f
    * Force vector of node to modify
    * 
    * @param sig
    * Computed material stress
    * 
    * @param dv
    * 3D displacement increment. detJ * integrationPt.weight
    * 
    * @param n
    * Node index
    * 
    * @param pt 
    * Integration point 
    * 
    * @param el 
    * Integration point's shell element
    * 
    * FEBio: FEElasticShellDomain::ElementInternalForce
    */
   public static void addShellStressForce (
      Vector3d f, Vector3d rf, SymmetricMatrix3d sig, double dv, int n, 
      ShellIntegrationPoint3d pt, ShellFemElement3d el) {
      
      double t = pt.coords.z;
      double N = pt.getShapeWeights ().get(n);
      double dNdr = pt.getShapeGradient ()[n].x;
      double dNds = pt.getShapeGradient ()[n].y;
      
      Vector3d[] gct = pt.getContraBaseVectors (el);
      
      Vector3d gradM = new Vector3d();
      gradM.scaledAdd(dNdr, gct[0]);
      gradM.scaledAdd(dNds, gct[1]);
      
      Vector3d gradMu = new Vector3d();
      gradMu.scaledAdd (1+t, gradM);
      gradMu.scaledAdd (N, gct[2]);
      gradMu.scale (0.5);
      
      Vector3d gradMd = new Vector3d();
      gradMd.scaledAdd (1-t, gradM);
      gradMd.scaledAdd (-N, gct[2]);
      gradMd.scale (0.5);
      
      Vector3d fu = new Vector3d(gradMu);
      sig.mul(fu);
      
      Vector3d fd = new Vector3d(gradMd);
      sig.mul(fd);
      
      // Increment displacement force. 
      f.x += fu.x*dv;
      f.y += fu.y*dv;
      f.z += fu.z*dv;
      
      // Increment angular force.
      rf.x += fd.x*dv;
      rf.y += fd.y*dv;
      rf.z += fd.z*dv;
      
      // In FEBio, force are subtracted here.
      // Artisynth does this subtraction later in
      // FemModel3d.updateNodeForces().
   }
   
   /**
    * Add inertia stiffness between a pair of nodes. This is specific for
    * shell-elements.
    * 
    * FEBio: FEElasticShellDomain::InertialForces
    * 
    * @param K
    * 6x6 (i.e. 6dof) stiffness matrix to add the stiffness into.
    * 
    * @param el
    * Shell element that iPt, i, and j belong to.
    * 
    * @param iPt
    * Integration point of the pair of nodes.
    * 
    * @param i
    * First node index of pair.
    * 
    * @param j
    * Second node index of pair.
    * 
    * @param scale
    * Scaler for added stiffness. Use 1.0 unless timestep is fractured into 
    * smaller integration steps (e.g. 1.0 / beta*h*h).
    */
   public static void addShellInertialStiffness(Matrix6d K, 
      ShellFemElement3d el, ShellIntegrationPoint3d iPt, int i, int j, 
      double scale) {
      
      double density = el.getDensity();
      
      VectorNd Ns = iPt.getShapeWeights();
      
      iPt.computeJacobian0(el);
      double detJ0 = iPt.getJ().determinant () * iPt.getWeight();
      
      double t = iPt.coords.z;
      
      double iN = Ns.get(i);
      double jN = Ns.get(i);
         
      double Kuu = (1+t)/2.0*iN*(1+t)/2.0*jN*scale*density*detJ0;
      double Kud = (1+t)/2.0*iN*(1-t)/2.0*jN*scale*density*detJ0;
      double Kdu = (1-t)/2.0*iN*(1+t)/2.0*jN*scale*density*detJ0;
      double Kdd = (1-t)/2.0*iN*(1-t)/2.0*jN*scale*density*detJ0;
         
      K.m00 += Kuu;
      K.m11 += Kuu;
      K.m22 += Kuu;
      
      K.m03 += Kud;
      K.m14 += Kud;
      K.m25 += Kud;
      
      K.m30 += Kdu;
      K.m41 += Kdu;
      K.m52 += Kdu;
      
      K.m33 += Kdd;
      K.m44 += Kdd;
      K.m55 += Kdd;
   }
   
   
   /**
    * Add inertia forces to each node for a particular shell element.
    * 
    * @param el
    * Shell-element to have its node forces summed with inertia forces.
    * 
    * @postcond
    * node.myInternalForce and node.myInternalDirForce are increased with 
    * inertia forces.
    */
   public static void addShellInertiaForces(ShellFemElement3d el) {
      double density = el.getDensity();
      
      // For each integration point...
      ShellIntegrationPoint3d[] iPts = el.getIntegrationPoints();
      for (int k = 0; k < iPts.length; k++) {
         ShellIntegrationPoint3d iPt = iPts[k];
      
         iPt.computeJacobian0(el);
         double detJ0 = iPt.getJ().determinant ()*iPt.getWeight();
         
         VectorNd Ns = iPt.getShapeWeights();
         
         double t = iPt.coords.z;

         // For each node...
         for (int n = 0; n < el.numNodes(); n++) {
            ShellFemNode3d sn = (ShellFemNode3d)el.myNodes[n];
            
            // TODO
            Vector3d a = null;
            
            double N = Ns.get(n);
            
            Vector3d fu = new Vector3d(a);
            fu.scale( density*N*(1+t)/2.0*detJ0 );
            
            Vector3d fd = new Vector3d(a);
            fd.scale( density*N*(1-t)/2.0*detJ0 );
            
            sn.myInternalForce.add( fu );
            sn.myInternalDirForce.add( fd );
         }
      }
      
      throw new RuntimeException("Unimplemented");
   }
}
