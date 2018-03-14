package artisynth.models.carapas;

import maspack.matrix.Vector3d;
import maspack.properties.PropertyList;
import artisynth.core.mechmodels.AxialSpring;

public class Forceps extends AxialSpring {
   
   private Vector3d tighteningForce;
   private Vector3d currentU;
   
   double tightening = 0;
   double maximumTighteningMagnitude = 10000;

   static PropertyList forcepsProps = new PropertyList(Forceps.class, AxialSpring.class);
   
   @Override
   public PropertyList getAllPropertyInfo() {
      return forcepsProps;
   }
   
   static {
      forcepsProps.add ("tightening", "Forceps Tightening", 0);
      forcepsProps.add ("maximumTighteningMagnitude", "Maximum Forceps Tightening Magnitude", 1000000);
   }
   
   public Forceps() {
      super();
      this.currentU = new Vector3d();
      this.tighteningForce = new Vector3d();
   }
   
   public double getTightening () {
      return tightening;
   }

   public void setTightening (double tightening) {
      this.tightening = tightening;
   }
   
   public double getMaximumTighteningMagnitude () {
      return maximumTighteningMagnitude;
   }

   public void setMaximumTighteningMagnitude (double maximumTighteningMagnitude) {
      this.maximumTighteningMagnitude = maximumTighteningMagnitude;
   }

   public void computeForce(Vector3d f) {
      updateU();
      
      this.currentU.sub(myPnt1.getPosition (), myPnt0.getPosition ());
      
      this.tighteningForce.scale (this.tightening/100.0, this.myU);
      
      f.set(tighteningForce.scale( maximumTighteningMagnitude));
   }
}
