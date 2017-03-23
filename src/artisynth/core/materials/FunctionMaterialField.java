package artisynth.core.materials;

import maspack.function.Function3x1;
import maspack.matrix.Point3d;

/**
 * Field based on a 3x1 function, applied to the rest (Lagrangian)
 * coordinates of a material
 *
 */
public class FunctionMaterialField implements MaterialField {

   Function3x1 myFunc;
   
   /**
    * Field based on
    * @param func
    */
   public FunctionMaterialField(Function3x1 func) {
      this.myFunc = func;
   }
   
   @Override
   public double size() {
      return 1;
   }

   @Override
   public double eval(MaterialCoordinate coord, int dim) {
      Point3d pos = coord.getRestPosition();
      return myFunc.eval(pos);
   }
   
   @Override
   public boolean equals(Object obj) {
      if (obj == this) {
         return true;
      }
      if (obj == null || !(obj instanceof Function3x1)) {
         return false;
      }
      FunctionMaterialField other = (FunctionMaterialField)obj;
      return other.myFunc == myFunc;
   }

   @Override
   public MaterialField clone() {
      FunctionMaterialField out;
      try {
         out = (FunctionMaterialField)super.clone();
      } catch (CloneNotSupportedException e) {
         e.printStackTrace();
         return null;
      }
      return out;
   }

}
