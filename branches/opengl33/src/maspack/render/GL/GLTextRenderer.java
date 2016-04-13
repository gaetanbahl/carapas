package maspack.render.GL;

import java.awt.geom.Point2D;
import java.nio.ByteBuffer;
import java.util.ArrayList;

import javax.media.opengl.GL;

import maspack.render.TextImageStore.GlyphStore;
import maspack.render.TextureTextRenderer;

public class GLTextRenderer extends TextureTextRenderer {

   GLTexture texture;
   GLPipelineRenderer renderer;

   public GLTextRenderer (GLTexture tex, GLPipelineRenderer renderer,
      int preferredWidth, int preferredHeight, int maxDimension) {
      this (
         tex, renderer, preferredWidth, preferredHeight, maxDimension,
         DEFAULT_ANTIALIASING);
   }

   public GLTextRenderer (GLTexture tex, GLPipelineRenderer renderer,
   int preferredWidth, int preferredHeight, int maxDimension,
   boolean antialiasing) {
      super(preferredWidth, preferredHeight, maxDimension, antialiasing);
      this.texture = tex.acquire ();
      this.renderer = renderer;
   }

   /**
    * Bind's internal texture
    * @param gl
    */
   public void bind(GL gl) {
      texture.bind (gl);
   }
   
   public void unbind(GL gl) {
      texture.unbind (gl);
   }

   /**
    * Prepares for drawing
    * 
    * @param gl
    */
   public void begin (GL gl) {
      super.begin();

      // make sure texture is bound
      texture.bind (gl);

      // ready for 100 squares
      renderer.setup (true, false, true);
      renderer.begin (gl, GL.GL_TRIANGLES, 6 * 100);
      renderer.normal (0, 0, 1);
   }
   
   public GLTexture getTexture() {
      return texture;
   }
  
   @Override
   protected void render (GlyphStore store, float[] origin, float scale) {

      Point2D loc = store.getBaselineLocation (); // relative to origin
      float x1 =
         (float)((loc.getX () - store.getBaselineX ()) * scale) + origin[0];
      float y1 =
         (float)((loc.getY () - store.getBaselineY ()) * scale) + origin[1];
      float z = origin[2];

      int w = store.getWidth ();
      int h = store.getHeight ();

      int width = getWidth ();
      int height = getHeight ();

      float tx1 = (float)(store.getLeft ()) / width;
      float ty1 = (float)(store.getBottom ()) / height;
      // works better adding full width/height than the "true" one -1
      // must be the way texture coordinates are working
      float tx2 = (float)(store.getLeft () + store.getWidth ()) / width;  
      float ty2 = (float)(store.getBottom () + store.getHeight ()) / height;

      float x2 = x1 + scale * (w - 1);
      float y2 = y1 + scale * (h - 1);
      
      // two triangles
      renderer.texcoord (tx1, ty1);
      renderer.vertex (x1, y1, z);
      renderer.texcoord (tx2, ty2);
      renderer.vertex (x2, y2, z);
      renderer.texcoord (tx1, ty2);
      renderer.vertex (x1, y2, z);

      renderer.texcoord (tx2, ty2);
      renderer.vertex (x2, y2, z);
      renderer.texcoord (tx1, ty1);
      renderer.vertex (x1, y1, z);
      renderer.texcoord (tx2, ty1);
      renderer.vertex (x2, y1, z);
      
      GLSupport.checkAndPrintGLError (renderer.getGL ());

   }

   @Override
   protected ArrayList<GlyphQueue> draw (ArrayList<GlyphQueue> queue) {
      ArrayList<GlyphQueue> remaining = super.draw (queue);
      renderer.flush ();  // draw primitives before texture changes
      return remaining;
   }
   
   @Override
   protected void replaceSubTexture (
      int x, int y, int width, int height, ByteBuffer buff) {
      GL gl = renderer.getGL ();
      gl.glTexSubImage2D (
         texture.getTarget (), 0, x, y, width,
         height, GL.GL_RGBA, GL.GL_UNSIGNED_BYTE, buff);
      gl.glGenerateMipmap (texture.getTarget ());
      GLSupport.checkAndPrintGLError (gl);  
   }
   
   @Override
   protected void replaceTexture (int width, int height, ByteBuffer buff) {
      
      GL gl = renderer.getGL ();
      GLSupport.checkAndPrintGLError (gl);
      gl.glTexImage2D (
         texture.getTarget (), 0, GL.GL_RGBA, width,
         height, 0, GL.GL_RGBA, GL.GL_UNSIGNED_BYTE, buff);
      gl.glGenerateMipmap (texture.getTarget ());
      GLSupport.checkAndPrintGLError (gl);
   }

   /**
    * Commits any changes to the graphics pipeline
    * 
    * @param gl
    */
   public void end (GL gl) {
      flush ();
      renderer.end ();
      super.end ();
   }

   public void dispose (GL gl) {
      if (texture != null) {
         texture.dispose (gl);
         texture = null;
      }
      
      if (renderer != null) {
         renderer.dispose (gl);
         renderer = null;
      }
   }

   public static GLTextRenderer generate (
      GL gl, GLPipelineRenderer renderer, int preferredWidth,
      int preferredHeight) {

      GLTexture tex = GLTexture.generate (gl, GL.GL_TEXTURE_2D);
      int[] v = new int[1];
      gl.glGetIntegerv (GL.GL_MAX_TEXTURE_SIZE, v, 0);
      
      return new GLTextRenderer (
         tex, renderer, preferredWidth, preferredHeight, v[0]);
   }

   public static GLTextRenderer generate (GL gl, GLPipelineRenderer renderer) {

      return generate(gl, renderer, DEFAULT_SIZE, DEFAULT_SIZE);
   }

}
