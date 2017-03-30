package artisynth.core.materials;

import artisynth.core.util.ScalableUnits;
import maspack.matrix.Matrix3d;
import maspack.matrix.Matrix6d;
import maspack.matrix.SymmetricMatrix3d;
import maspack.util.Clonable;

/**
 * Interface for constitutive materials, including FemMaterial
 * and muscle materials. The work-horses are computeTangent(...) 
 * and computeStress(...)
 *
 */
public interface ConstitutiveMaterial extends ScalableUnits, Clonable {

   /**
    * Computes the tangent stiffness matrix
    * @param D tangent stiffness, populated
    * @param stress the current stress tensor
    * @param def deformation information, includes deformation gradient and pressure
    * @param Q coordinate frame specifying directions of anisotropy
    * @param baseMat underlying base material (if any)
    */
   public void computeTangent (
      Matrix6d D, SymmetricMatrix3d stress, SolidDeformation def, 
      Matrix3d Q, FemMaterial baseMat);
   
   /**
    * Computes the strain tensor given the supplied deformation
    * @param sigma strain tensor, populated
    * @param def deformation information, includes deformation gradient and pressure
    * @param Q coordinate frame specifying directions of anisotropy
    * @param baseMat underlying base material (if any)
    */
   public void computeStress (
      SymmetricMatrix3d sigma, SolidDeformation def, Matrix3d Q,
      FemMaterial baseMat);

   /** 
    * Returns true if the tangent matrix for this material is symmetric.  While
    * this is normally true, some special materials (such as those whose stress
    * is not derived from a conservative energy funtion) may have a non-symmetric
    * tangent, in which case this function should be overridden to return false.
    *
    * @return true if the tangent matrix for this material is symmetric.
    */
   public boolean hasSymmetricTangent();
   
   /**
    * Returns true if this material is defined for a deformation gradient
    * with a non-positive determinant.
    */
   public boolean isInvertible();

   /**
    * Supports incompressibility
    * @return
    */
   public boolean isIncompressible();
   
   /**
    * Retrieves incompressible behaviour characteristics if incompressible,
    * null otherwise.
    * @return incompressible info
    */
   public BulkIncompressibleBehavior getIncompressibleBehavior();

   /**
    * Signals that the material is to use a small-strain assumption ( J ~ I )
    * @return true if linear, false otherwise
    */
   public boolean isLinear();
   /**
    * Signals that the material uses a corotated formulation, where
    * the rotational component of the strain is removed.  This is mainly used
    * in conjunction with the small-strain assumption
    * @return true if corotated, false otherwise
    */
   public boolean isCorotated();
   
   /**
    * Contains viscoelastic properties
    * @return true if visco
    */
   public boolean isViscoelastic();

   /**
    * Retrieves viscoelastic behaviour characteristics if available,
    * null otherwise.
    * @return incompressible info
    */
   public ViscoelasticBehavior getViscoBehavior();
   
   @Override
   public ConstitutiveMaterial clone();
   
}
