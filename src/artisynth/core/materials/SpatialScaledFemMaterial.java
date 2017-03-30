package artisynth.core.materials;

import maspack.matrix.Matrix3d;
import maspack.matrix.Matrix6d;
import maspack.matrix.SymmetricMatrix3d;
import maspack.properties.PropertyList;

/**
 * Wrapper around a FemMaterial that scales the stress and tangent components
 * according to a spatially-dependent scale factor
 *
 */
public class SpatialScaledFemMaterial extends FemMaterial {
   
   static {
      FemMaterial.registerSubclass(SpatialScaledFemMaterial.class);
   }

   private static FemMaterial DEFAULT_MATERIAL = new LinearMaterial();
   private static MaterialField DEFAULT_SCALE_FIELD = new ConstantMaterialField(new double[]{1}); 
   
   FemMaterial myMaterial;
   MaterialField myScaleField;

   public static PropertyList myProps =
      new PropertyList (SpatialScaledFemMaterial.class, FemMaterial.class);

   static {
      myProps.add("material", "underlying material", DEFAULT_MATERIAL, "CE");
   }
   
   public PropertyList getAllPropertyInfo() {
      return myProps;
   }
   
   public SpatialScaledFemMaterial() {
      this(DEFAULT_MATERIAL, DEFAULT_SCALE_FIELD);
   }
   
   public SpatialScaledFemMaterial(FemMaterial mat, MaterialField scaleField) {
      this.myScaleField = scaleField;
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
   
   public void setScaleField(MaterialField field) {
      myScaleField = field;
      notifyHostOfPropertyChange("scale");
   }
   
   public MaterialField getScaleField() {
      return myScaleField;
   }
   
   /**
    * Evaluates the scale field at the supplied coordinate
    * @param coord
    * @return
    */
   public double getScale(MaterialCoordinate coord) {
      return myScaleField.eval(coord, 0);
   }

   @Override
   public void computeTangent(
      Matrix6d D, SymmetricMatrix3d stress, SolidDeformation def, Matrix3d Q,
      FemMaterial baseMat) {
      myMaterial.computeTangent(D, stress, def, Q, baseMat);
      double scale = myScaleField.eval(def.getMaterialCoordinate(), 0);
      D.scale(scale);
   }

   @Override
   public void computeStress(
      SymmetricMatrix3d sigma, SolidDeformation def, Matrix3d Q,
      FemMaterial baseMat) {
      myMaterial.computeStress(sigma, def, Q, baseMat);
      double scale = myScaleField.eval(def.getMaterialCoordinate(), 0);
      sigma.scale(scale);
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
      if (!(mat instanceof SpatialScaledFemMaterial)) {
         return false;
      }
      SpatialScaledFemMaterial smat = (SpatialScaledFemMaterial)mat;
      if (!(myMaterial.equals(smat.myMaterial))) {
         return false;
      }
      if (!myScaleField.equals(smat.myScaleField)) {
         return false;
      }
      return super.equals(mat);
   }

   @Override
   public SpatialScaledFemMaterial clone() {
      SpatialScaledFemMaterial copy = (SpatialScaledFemMaterial)super.clone();
      copy.myMaterial = myMaterial.clone();
      copy.myScaleField = myScaleField.clone();
      return copy;
   }
   
}
