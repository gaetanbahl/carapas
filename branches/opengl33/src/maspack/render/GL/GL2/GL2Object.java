package maspack.render.GL.GL2;

import javax.media.opengl.GL2;

public class GL2Object implements GL2Drawable {
   
   GL2DisplayList displayList;
   
   public GL2Object(GL2DisplayList displayList) {
      this.displayList = displayList.acquire ();  // hold on to a reference of the displaylist
      System.out.println ("GL2Object born (" + this + ")");
   }

   @Override
   public void draw (GL2 gl) {
      displayList.executeAll (gl);
   }
   
   public GL2DisplayList getDisplayList() {
      return displayList;
   }
   
   public void beginCompile(GL2 gl) {
      displayList.compile (gl);
   }
   
   public void endCompile(GL2 gl) {
      displayList.end (gl);
   }
   
   public void beginCompileAndDraw(GL2 gl) {
      displayList.compileAndExecute (gl);
   }

   @Override
   public void dispose () {
      if (displayList != null) {
         System.out.println ("GL2Object disposed (" + this + ")");
         displayList.release ();  // release reference so it can be freed
         displayList = null;
      }
   }
   
   @Override
   protected void finalize () throws Throwable {
      System.out.println ("GL2Object dying (" + this + ")");
      dispose();
   }

   @Override
   public boolean isValid () {
      if (displayList != null && displayList.isValid()) {
         return true;
      }
      return false;
   }

   @Override
   public boolean isDisposed () {
      return (displayList == null);
   }
   
   @Override
   public boolean disposeInvalid () {
      if (isDisposed ()) {
         return true;
      }
      if (!isValid ()) {
         dispose();
         return true;
      }
      return false;
   }

}