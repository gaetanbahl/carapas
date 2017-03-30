package artisynth.core.materials;

import maspack.matrix.Matrix3d;
import maspack.matrix.Matrix6d;
import maspack.matrix.SymmetricMatrix3d;
import maspack.properties.PropertyList;

/**
 * Wrapper around a FemMaterial that scales the stress and tangent components
 *
 */
public class ScaledFemMaterial extends FemMaterial {
   
   static {
      FemMaterial.registerSubclass(ScaledFemMaterial.class);
   }

   private static FemMaterial DEFAULT_MATERIAL = new LinearMaterial();
   private static double DEFAULT_SCALE = 1.0; 
   
   FemMaterial myMaterial;
   double myScale;

   public static PropertyList myProps =
      new PropertyList (ScaledFemMaterial.class, FemMaterial.class);

   static {
      myProps.add("material", "underlying material", DEFAULT_MATERIAL, "CE");
      myProps.add("scale", "stress and tangent scale", DEFAULT_SCALE);
   }
   
   public PropertyList getAllPropertyInfo() {
      return myProps;
   }
   
   public ScaledFemMaterial() {
      this(DEFAULT_MATERIAL, DEFAULT_SCALE);
   }
   
   public ScaledFemMaterial(FemMaterial mat, double scale) {
      this.myScale = scale;
      this.myMaterial = mat;
   }
   
   public FemMaterial getMaterial() {
      return myMaterial;
   }
   
   public void setMaterial(FemMaterial mat) {
      myMaterial = (FemMaterial)MaterialBase.updateMaterial (
         this, "material", myMaterial, mat);
      notifyHostOfPropertyChange("material");
   }
   
   public void setScale(double scale) {
      myScale = scale;
      notifyHostOfPropertyChange("scale");
   }
   
   public double getScale() {
      return myScale;
   }

   @Override
   public void computeTangent(
      Matrix6d D, SymmetricMatrix3d stress, SolidDeformation def, Matrix3d Q,
      FemMaterial baseMat) {
      myMaterial.computeTangent(D, stress, def, Q, baseMat);
      D.scale(myScale);
   }

   @Override
   public void computeStress(
      SymmetricMatrix3d sigma, SolidDeformation def, Matrix3d Q,
      FemMaterial baseMat) {
      myMaterial.computeStress(sigma, def, Q, baseMat);
      sigma.scale(myScale);
   }
   
   @Override
   public boolean isLinear() {
      return myMaterial.isLinear();
   }
   
   @Override
   public boolean isCorotated() {
      return myMaterial.isCorotated();
   }
   
   @Override
   public boolean isInvertible() {
      return myMaterial.isInvertible();
   }
   
   @Override
   public boolean isIncompressible() {
      return myMaterial.isIncompressible();
   }
   
   @Override
   public BulkIncompressibleBehavior getIncompressibleBehavior() {
      return myMaterial.getIncompressibleBehavior();
   }
   
   @Override
   public boolean hasSymmetricTangent() {
      return myMaterial.hasSymmetricTangent();
   }
   
   @Override
   public boolean isViscoelastic() {
      return myMaterial.isViscoelastic();
   }
   
   @Override
   public ViscoelasticBehavior getViscoBehavior() {
      return myMaterial.getViscoBehavior();
   }
   
   @Override
   public void setViscoBehavior(ViscoelasticBehavior veb) {
      myMaterial.setViscoBehavior(veb);
   }
   
   @Override
   public void scaleMass(double s) {
      myMaterial.scaleMass(s);
   }
   
   @Override
   public void scaleDistance(double s) {
      myMaterial.scaleDistance(s);
   }
   
   @Override
   public boolean equals(FemMaterial mat) {
      if (!(mat instanceof ScaledFemMaterial)) {
         return false;
      }
      ScaledFemMaterial smat = (ScaledFemMaterial)mat;
      if (!(myMaterial.equals(smat.myMaterial))) {
         return false;
      }
      if (myScale != smat.myScale) {
         return false;
      }
      return super.equals(mat);
   }

   @Override
   public ScaledFemMaterial clone() {
      ScaledFemMaterial copy = (ScaledFemMaterial)super.clone();
      copy.myMaterial = myMaterial.clone();
      copy.myScale = myScale;
      return copy;
   }

   // Sanchez, March 27, 2013
   // useful for separating incompressibility stuff from computeStressAndStiffness() function
   public void computePressureStress(SymmetricMatrix3d sigma, double p) {
      sigma.setZero();
      sigma.m00 = p;
      sigma.m11 = p;
      sigma.m22 = p;
   }
   
   public void addPressureStress(SymmetricMatrix3d sigma, double p) {
      sigma.m00 += p;
      sigma.m11 += p;
      sigma.m22 += p;
   }

   public void computePressureTangent(Matrix6d D, double p) {
      D.setZero();
      TensorUtils.addScaledIdentityProduct (D, p);
      TensorUtils.addScaledIdentity (D, -2*p);
      D.setLowerToUpper();
   }
   
   public void addPressureTangent(Matrix6d D, double p) {
      TensorUtils.addScaledIdentityProduct (D, p);
      TensorUtils.addScaledIdentity (D, -2*p);
   }
   
}
