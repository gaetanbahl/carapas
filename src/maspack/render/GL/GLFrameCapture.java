package maspack.render.GL;

import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.Future;

import javax.imageio.ImageIO;
import javax.media.opengl.GL2;
import javax.media.opengl.GL2GL3;

import maspack.concurrency.SimpleThreadManager;
import maspack.render.GL.GLViewer.ScreenShotCallback;

public class GLFrameCapture {

   private static class ImageSaverCallback implements ScreenShotCallback {
      File file;
      String format;
      
      public ImageSaverCallback(File file, String fmt) {
         this.file = file;
         this.format = fmt;
      }

      @Override
      public void screenshot(BufferedImage image) {
         try {
            ImageIO.write (image, format, file);
         }
         catch (IOException io_e) {
            io_e.printStackTrace();
         }
      }
      
   }
   
   SimpleThreadManager imageThreadManager;
   
   
   private ScreenShotCallback callback;
   private FrameBufferObject fbo;
   private volatile boolean lock;
   
   private GLFrameCapture(int x, int y, int w, int h, 
      int nsamples, boolean gammaCorrected,
      ScreenShotCallback callback) {
      
      fbo = new FrameBufferObject(x, y, w, h, nsamples, gammaCorrected);
      lock = false;
      this.callback = callback;
   }
   
   public GLFrameCapture(
      int w, int h, int nsamples, boolean gammaCorrected, 
      ScreenShotCallback callback) {
      this(0, 0, w, h, nsamples, gammaCorrected, callback);
   }
   
   public GLFrameCapture(
      int w, int h, int nsamples, boolean gammaCorrected, 
      File file, String format) {
      this(0,0,w,h, nsamples, gammaCorrected, file,format);
   }
  
   public GLFrameCapture(
      int x, int y, int w, int h, int nsamples, boolean gammaCorrected, 
      File file, String format) {
      this(x, y, w, h, nsamples, gammaCorrected, 
         new ImageSaverCallback(file, format));
   }
   
   /**
    * @return underlying frame buffer object
    */
   public FrameBufferObject getFBO() {
      return fbo;
   }
   
   /**
    * Don't allow two threads (or same thread twice) locking this object
    */
   public synchronized void lock() {
      int count = 0;
      final int COUNT_LIMIT = 1000;
      while (lock && count < COUNT_LIMIT) {
         // poll until free
         // XXX sometimes doesn't seem to be unlocked?!?!
         try {
            Thread.sleep(10);
         } catch (InterruptedException e) {}
         ++count;
         Thread.yield();
      }
      if (count > COUNT_LIMIT) {
         System.err.println("Frame capture lock timeout");
      }
      lock = true;
   }
   
   public synchronized void unlock() {
      lock = false;
   }

   public void reconfigure(GL2GL3 gl, int w, int h, int nsamples, boolean gammaCorrection, 
      File file, String format) {
      reconfigure(gl, 0, 0, w, h, nsamples, gammaCorrection, new ImageSaverCallback(file, format));
   }
   
   public void reconfigure(GL2GL3 gl, int x, int y, int w, int h, int nsamples, boolean gammaCorrection, 
      File file, String format) {
      reconfigure(gl, x, y, w, h, nsamples, gammaCorrection, new ImageSaverCallback(file, format));
   }
   
   public void reconfigure(GL2GL3 gl, int w, int h, int nsamples, 
      boolean gammaCorrection, ScreenShotCallback callback) {
      reconfigure(gl, 0, 0, w, h, nsamples, gammaCorrection, callback);
   }
   
   public void reconfigure(GL2GL3 gl, int x, int y, int w, int h, int nsamples, 
      boolean gammaCorrection, ScreenShotCallback callback) {
      fbo.configure(gl, x, y, w, h, nsamples, gammaCorrection);
      this.callback = callback;
   }
   
   public void activateFBO(GL2GL3 gl) {
      fbo.activate(gl);
      gl.glClear (GL2.GL_COLOR_BUFFER_BIT | GL2.GL_DEPTH_BUFFER_BIT);
   }
   
   public void deactivateFBO(GL2GL3 gl) {
      fbo.deactivate(gl);
   }
   
   private static class CallbackRunnable implements Runnable {
      
      BufferedImage image;
      ScreenShotCallback callback;
      
      public CallbackRunnable(ScreenShotCallback callback, BufferedImage image) {
         this.callback = callback;
         this.image = image;
      }
      
      public void run() {
        callback.screenshot(image);
      }
   }
   
   /**
    * Captures an image of the canvas and saves it to the specified file.
    */
   public void capture (GL2GL3 gl) {

      // Get the ARGB pixels as integers.
      int[] pixelsARGB = fbo.getPixelsARGB (gl);
      
      // JPG for some reason breaks if we turn on the alpha layer.
      // JPG doesn't support alpha anyways.
      BufferedImage image = createImage(fbo.getWidth(), fbo.getHeight(), pixelsARGB);
      
      if (imageThreadManager == null) {
         imageThreadManager = new SimpleThreadManager("GLFrameCapture", 1, 5000);
      }
      
      // write image in separate thread
      synchronized (imageThreadManager) {
         imageThreadManager.execute(new CallbackRunnable(callback, image));  
      }
      
   }
   
   private static BufferedImage createImage(int width, int height, int[] pixelsARGB) {
     
      BufferedImage image = new BufferedImage (width, height, BufferedImage.TYPE_INT_RGB);
      image.setRGB (0, 0, width, height, pixelsARGB, 0, width);
      return image;
   }
   
   public BufferedImage captureImage(GL2GL3 gl) {
      // Get the ARGB pixels as integers.
      int[] pixelsARGB = fbo.getPixelsARGB (gl);
      
      return createImage(fbo.getWidth(), fbo.getHeight(), pixelsARGB);
   }
   
   public void waitForCompletion() {
      synchronized (imageThreadManager) {
         while (imageThreadManager.hasNextFuture()) {
            Future<?> fut = imageThreadManager.popFuture();
            try {
               fut.get();
            } catch (InterruptedException | ExecutionException e) {
               e.printStackTrace();
            }
         }
      }
   }
   
   public void dispose(GL2GL3 gl) {
      fbo.dispose(gl);
   }
   
}
