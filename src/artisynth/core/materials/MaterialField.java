package artisynth.core.materials;

import maspack.util.Clonable;

/**
 * A field that can be evaluated at a specified material coordinate,
 * useful for creating spatially-varying material properties
 * @author Antonio
 *
 */
public interface MaterialField extends Clonable {

   /**
    * @return number of dimensions in the field
    */
   public double size();
   
   /**
    * Evaluates the field at a given coordinate
    * @param coord spatial material coordinate
    * @param dim dimension of index to evaluate
    * @return the field value
    */
   public double eval(MaterialCoordinate coord, int dim);
   
   public MaterialField clone();
   
}
