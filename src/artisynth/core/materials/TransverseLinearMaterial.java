package artisynth.core.materials;

import maspack.matrix.Matrix3d;
import maspack.matrix.Matrix3dBase;
import maspack.matrix.Matrix6d;
import maspack.matrix.RotationMatrix3d;
import maspack.matrix.SVDecomposition;
import maspack.matrix.SymmetricMatrix3d;
import maspack.matrix.Vector2d;
import maspack.properties.PropertyList;

public class TransverseLinearMaterial extends LinearMaterialBase {
   
   static {
      FemMaterial.registerSubclass(TransverseLinearMaterial.class);
   }
   
   private static Vector2d DEFAULT_YOUNGS_MODULUS = new Vector2d(LinearMaterial.DEFAULT_E, LinearMaterial.DEFAULT_E);
   private static Vector2d DEFAULT_POISSONS_RATIO = new Vector2d(LinearMaterial.DEFAULT_NU, LinearMaterial.DEFAULT_NU);
   private static double DEFAULT_SHEAR_MODULUS = LinearMaterial.DEFAULT_E/2/(1+LinearMaterial.DEFAULT_NU);
   
   private Matrix6d myC;        // anisotropic stiffness matrix
   private Vector2d myE;        // radial, z-axis young's modulus
   private double myG;          // shear modulus
   private Vector2d myNu;       // radial, z-axis Poisson's ratio
   private boolean stiffnessValid;
   
   public static PropertyList myProps =
      new PropertyList (TransverseLinearMaterial.class, LinearMaterialBase.class);

   static {
      myProps.add ("youngsModulus", "radial and z-axis Young's modulus,", DEFAULT_YOUNGS_MODULUS);
      myProps.add ("shearModulus", "radial-to-z-axis shear modulus,", DEFAULT_SHEAR_MODULUS);
      myProps.add ("poissonsRatio", "radial and z-axis Young's modulus,", DEFAULT_POISSONS_RATIO);
   }

   public PropertyList getAllPropertyInfo() {
      return myProps;
   }

   public TransverseLinearMaterial () {
      this(DEFAULT_YOUNGS_MODULUS, DEFAULT_SHEAR_MODULUS, DEFAULT_POISSONS_RATIO, LinearMaterial.DEFAULT_COROTATED);
   }

   public TransverseLinearMaterial (Vector2d E, double G, Vector2d nu, boolean corotated) {
      super(corotated);
      
      myE = new Vector2d(E);
      myG = G;
      myNu = new Vector2d(nu);
      myC = null;
      stiffnessValid = false;
   }

   public Matrix6d getStiffnessTensor() {
      maybeUpdateStiffness();
      return myC;
   }
   
   protected void updateStiffnessTensor() {
      Matrix6d invC = new Matrix6d();
      invC.m00 = 1.0/myE.x;
      invC.m01 = -myNu.x/myE.x;
      invC.m02 = -myNu.y/myE.y;
      
      invC.m10 = invC.m01;
      invC.m11 = invC.m00;
      invC.m12 = invC.m02;
      
      invC.m20 = invC.m02;
      invC.m21 = invC.m20;
      invC.m22 = 1.0/myE.y;
      
      invC.m33 = 2*(1+myNu.x)/myE.x;
      invC.m44 = 1.0/myG;
      invC.m55 = invC.m44;
      
      SVDecomposition svd = new SVDecomposition(invC);
      
      if (myC == null) {
         myC = new Matrix6d();
      }
      svd.pseudoInverse(myC);
      
      Matrix6d C = AnisotropicLinearMaterial.createIsotropicStiffness((myE.x + myE.y)/2, (myNu.x + myNu.y)/2);
      Matrix6d Cinv = new Matrix6d();
      svd.factor(C);
      svd.pseudoInverse(Cinv);
      
      if (!C.epsilonEquals(myC, 1e-6)) {
         System.out.println("Hmm...");
         System.out.println(C);
         System.out.println(" vs ");
         System.out.println(myC);
      }
   }
   
   /**
    * Set Young's modulus, xy-plane (radial) and along z-axis (axial)
    * @param E young's modulus, E.x radial, E.y axial 
    */
   public void setYoungsModulus(Vector2d E) {
      setYoungsModulus(E.x, E.y);
   }
   
   /**
    * Get Young's modulus
    * @return
    */
   public Vector2d getYoungsModulus() {
      return myE;
   }
   
   /**
    * Sets the Youngs modulus
    * @param radial along xy-plane
    * @param axial along z-axis
    */
   public void setYoungsModulus(double radial, double axial) {
      myE.set(radial, axial);
      stiffnessValid = false;
      notifyHostOfPropertyChange("youngsModulus");
   }
   
   /**
    * Sets the shear modulus between xy and z
    * @param G shear modulus Gxz=Gyz
    */
   public void setShearModulus(double G) {
      myG = G;
      stiffnessValid = false;
      notifyHostOfPropertyChange("shearModulus");
   }
   
   /**
    * Gets the shear modulus betwen xy and z
    * @return shear modulus Gxz=Gyz
    */
   public double getShearModulus() {
      return myG;
   }
   
   /**
    * Sets Poisson's ratio (nu_xy, nu_xz=nu_yz) 
    * @return poisson's ratio
    */
   public void setPoissonsRatio(Vector2d nu) {
      setPoissonsRatio(nu.x, nu.y);
   }
   
   /**
    * Returns Poisson's ratio (nu_xy, nu_xz=nu_yz) 
    * @return poisson's ratio
    */
   public Vector2d getPoissonsRatio() {
      return myNu;
   }
   
   /**
    * Sets Poisson's ratio
    * @param nuxy in-plane ratio
    * @param nuxz out-of-plane ratio
    */
   public void setPoissonsRatio(double nuxy, double nuxz) {
      myNu.set(nuxy, nuxz);
      stiffnessValid = false;
      notifyHostOfPropertyChange("poissonsRatio");
   }
   
   protected void maybeUpdateStiffness() {
      if (!stiffnessValid) {
         updateStiffnessTensor();
      }
      stiffnessValid = true;
   }
   
  
   /** 
    * Computes the Cauchy stress from Cauchy strain and adds it to and existing
    * stress.
    * 
    * @param sigma value to which stress should be added
    * @param Eps Cauchy stress
    * @param R (optional) Co-Rotation matrix, if any
    */
   public void addStress (
      SymmetricMatrix3d sigma, SymmetricMatrix3d Eps, Matrix3dBase R) {
      maybeUpdateStiffness();

      // save for the rotated case
      double m00 = sigma.m00;
      double m11 = sigma.m11;
      double m22 = sigma.m22;
      double m01 = sigma.m01;
      double m02 = sigma.m02;
      double m12 = sigma.m12;

      if (R != null) {
         sigma.setZero();
      }

      // XXX Q?
      
      sigma.m00 += myC.m00*Eps.m00 + myC.m01*Eps.m11 + myC.m02*Eps.m22 + 2*myC.m03*Eps.m01 + 2*myC.m04*Eps.m12 + 2*myC.m05*Eps.m02;
      sigma.m11 += myC.m10*Eps.m00 + myC.m11*Eps.m11 + myC.m12*Eps.m22 + 2*myC.m13*Eps.m01 + 2*myC.m14*Eps.m12 + 2*myC.m15*Eps.m02;
      sigma.m22 += myC.m20*Eps.m00 + myC.m21*Eps.m11 + myC.m22*Eps.m22 + 2*myC.m23*Eps.m01 + 2*myC.m24*Eps.m12 + 2*myC.m25*Eps.m02;
      sigma.m01 += myC.m30*Eps.m00 + myC.m31*Eps.m11 + myC.m32*Eps.m22 + 2*myC.m33*Eps.m01 + 2*myC.m34*Eps.m12 + 2*myC.m35*Eps.m02;
      sigma.m12 += myC.m40*Eps.m00 + myC.m41*Eps.m11 + myC.m42*Eps.m22 + 2*myC.m43*Eps.m01 + 2*myC.m44*Eps.m12 + 2*myC.m45*Eps.m02;
      sigma.m02 += myC.m50*Eps.m00 + myC.m51*Eps.m11 + myC.m52*Eps.m22 + 2*myC.m53*Eps.m01 + 2*myC.m54*Eps.m12 + 2*myC.m55*Eps.m02;

      if (R != null) {
         sigma.mulLeftAndTransposeRight (R);

         sigma.m00 += m00;
         sigma.m11 += m11;
         sigma.m22 += m22;

         sigma.m01 += m01;
         sigma.m02 += m02;
         sigma.m12 += m12;

         sigma.m10 += m01;
         sigma.m20 += m02;
         sigma.m21 += m12;
      }

   }

   public void computeStress (
      SymmetricMatrix3d sigma, SolidDeformation def, Matrix3d Q, FemMaterial baseMat) {
      maybeUpdateStiffness();
      
      SymmetricMatrix3d eps  = new SymmetricMatrix3d();

      Matrix3dBase R = computeStrain(def.getF(), eps, def.getR());

      // XXX Q?
      
      // multiply
      sigma.m00 = myC.m00*eps.m00 + myC.m01*eps.m11 + myC.m02*eps.m22 + 2*myC.m03*eps.m01 + 2*myC.m04*eps.m12 + 2*myC.m05*eps.m02;
      sigma.m11 = myC.m10*eps.m00 + myC.m11*eps.m11 + myC.m12*eps.m22 + 2*myC.m13*eps.m01 + 2*myC.m14*eps.m12 + 2*myC.m15*eps.m02;
      sigma.m22 = myC.m20*eps.m00 + myC.m21*eps.m11 + myC.m22*eps.m22 + 2*myC.m23*eps.m01 + 2*myC.m24*eps.m12 + 2*myC.m25*eps.m02;
      sigma.m01 = myC.m30*eps.m00 + myC.m31*eps.m11 + myC.m32*eps.m22 + 2*myC.m33*eps.m01 + 2*myC.m34*eps.m12 + 2*myC.m35*eps.m02;
      sigma.m12 = myC.m40*eps.m00 + myC.m41*eps.m11 + myC.m42*eps.m22 + 2*myC.m43*eps.m01 + 2*myC.m44*eps.m12 + 2*myC.m45*eps.m02;
      sigma.m02 = myC.m50*eps.m00 + myC.m51*eps.m11 + myC.m52*eps.m22 + 2*myC.m53*eps.m01 + 2*myC.m54*eps.m12 + 2*myC.m55*eps.m02;
      sigma.m10 = sigma.m01;
      sigma.m20 = sigma.m02;
      sigma.m21 = sigma.m12;

      // rotate stress back to original frame
      if (isCorotated()) {
         sigma.mulLeftAndTransposeRight (R);
      }
   }

   public void computeTangent (
      Matrix6d D, SymmetricMatrix3d stress, SolidDeformation def, 
      Matrix3d Q, FemMaterial baseMat) {

      maybeUpdateStiffness();
      
      D.set(myC);
      
      // XXX Q?

      if (isCorotated()) {

         // need to rotate this tensor from linear frame into material one
         Matrix3d F = def.getF();
         Matrix3dBase dR = def.getR();
         RotationMatrix3d R = new RotationMatrix3d();
         if (dR == null) {
            R = computeRotation(F, null);
         } else {
            R.set(dR);
         }

         // R rotates from linear frame to the material one. Transpose
         // of R rotates from material frame to linear one.
         R.transpose();
         TensorUtils.rotateTangent2 (D, D, R);
      }
   }

   public boolean equals (FemMaterial mat) {
      if (!(mat instanceof TransverseLinearMaterial)) {
         return false;
      }
      TransverseLinearMaterial linm = (TransverseLinearMaterial)mat;
      if (!myC.equals(linm)) {
         return false;
      }
      else {
         return super.equals (mat);
      }
   }

   public TransverseLinearMaterial clone() {
      TransverseLinearMaterial mat = (TransverseLinearMaterial)super.clone();
      mat.myC = null;
      mat.stiffnessValid = false;
      mat.myE = myE.clone();
      mat.myNu = myNu.clone();
      mat.myG = myG;
      
      return mat;
   }

   @Override
   public void scaleDistance (double s) {
      if (s != 1) {
         super.scaleDistance (s);
         myE.scale(s);
         myG = myG*s;
         stiffnessValid = false;
      }
   }

   @Override
   public void scaleMass (double s) {
      if (s != 1) {
         super.scaleMass (s);
         myE.scale(s);
         myG = myG*s;
         stiffnessValid = false;
      }
   }

}
