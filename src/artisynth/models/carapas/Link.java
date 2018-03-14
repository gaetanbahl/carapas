package artisynth.models.carapas;

import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.RigidBody;



public class Link {
   FrameMarker p1, p2;

   public Link(RigidBody rb1, int idx1, RigidBody rb2, int idx2) {
       this(new FrameMarker(rb1.getMesh ().getVertex (idx1).getPosition ()), 
          new FrameMarker(rb2.getMesh ().getVertex (idx2).getPosition ()));
   }

   public Link(FrameMarker p1, FrameMarker p2) {
       this.p1 = p1;
       this.p2 = p2;
   }

   public void swap() {
      FrameMarker tmp = p1;
       p1 = p2;
       p2 = tmp;
   }

   public void sortByName() {
       if (p1.getFrame().getName().compareTo(p2.getFrame().getName()) > 0) {
           this.swap();
       }
   }

   public FrameMarker getFirst() {
       return p1;
   }

   public FrameMarker getSecond() {
       return p2;
   }

   @Override
   public String toString() {
       return "(" + p1 + " <-> " + p2 + ")";
   }

   @Override
   public boolean equals(Object o) {
       if (o instanceof Link) {
           return (p1.equals(((Link)o).p1) && p2.equals(((Link)o).p2))
               || (p2.equals(((Link)o).p1) && p1.equals(((Link)o).p2));
       }
       return false;
   }
}