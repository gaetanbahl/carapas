package artisynth.core.materials;

import java.util.Arrays;

/**
 * Constant multi-dimensional vector field
 */
public class ConstantMaterialField implements MaterialField {

   double[] vals;
   
   /**
    * Field value specified by the supplied constant vector
    * @param c
    */
   public ConstantMaterialField(double[] c) {
      set(c);
   }
   
   /**
    * Sets the constant values and size of the field.
    * @param c
    */
   public void set(double[] c) {
      this.vals = Arrays.copyOf(c, c.length);
   }
   
   @Override
   public double size() {
      return vals.length;
   }

   @Override
   public double eval(MaterialCoordinate coord, int dim) {
      return vals[dim];
   }
   
   @Override
   public boolean equals(Object obj) {
      if (obj == this) {
         return true;
      }
      if (obj == null || !(obj instanceof ConstantMaterialField)) {
         return false;
      }
      ConstantMaterialField other = (ConstantMaterialField)obj;
      if (!Arrays.equals(vals, other.vals)) {
         return false;
      }
      return true;
   }
   
   @Override
   public ConstantMaterialField clone()  {
      ConstantMaterialField out;
      try {
         out = (ConstantMaterialField) super.clone();
      } catch (CloneNotSupportedException e) {
         e.printStackTrace();
         return null;
      }
      out.vals = Arrays.copyOf(vals, vals.length);
      return out;
   }

}
