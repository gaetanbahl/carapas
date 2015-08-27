/**
 * Copyright (c) 2014, by the Authors: John E Lloyd (UBC), Antonio Sanchez (UBC),
 * and ArtiSynth Team Members
 *
 * This software is freely available under a 2-clause BSD license. Please see
 * the LICENSE file in the ArtiSynth distribution directory for details.
 */
package maspack.render.GL.GL2;

import java.awt.Dimension;
import java.awt.event.MouseWheelListener;
import java.io.File;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;

import javax.media.opengl.GL;
import javax.media.opengl.GL2;
import javax.media.opengl.GLAutoDrawable;
import javax.media.opengl.GLCapabilities;
import javax.media.opengl.GLProfile;
import javax.media.opengl.awt.GLCanvas;
import javax.swing.event.MouseInputListener;

import maspack.matrix.AffineTransform3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.properties.HasProperties;
import maspack.properties.PropertyList;
import maspack.render.Dragger3d;
import maspack.render.Material;
import maspack.render.RenderObject;
import maspack.render.RenderObject.RenderObjectVersion;
import maspack.render.RenderObject.VertexIndexSet;
import maspack.render.RenderProps;
import maspack.render.RenderProps.Faces;
import maspack.render.RenderProps.LineStyle;
import maspack.render.RenderProps.PointStyle;
import maspack.render.RenderProps.Shading;
import maspack.render.RenderableLine;
import maspack.render.RenderablePoint;
import maspack.render.Renderer;
import maspack.render.GL.GLClipPlane;
import maspack.render.GL.GLColorSelector;
import maspack.render.GL.GLFrameCapture;
import maspack.render.GL.GLGridPlane;
import maspack.render.GL.GLLight;
import maspack.render.GL.GLLight.LightSpace;
import maspack.render.GL.GLLight.LightType;
import maspack.render.GL.GLLightManager;
import maspack.render.GL.GLMouseAdapter;
import maspack.render.GL.GLOcclusionSelector;
import maspack.render.GL.GLSupport;
import maspack.render.GL.GLViewer;
import maspack.render.GL.GL2.DisplayListManager.DisplayListPassport;
import maspack.render.GL.GL2.RenderObjectKey.DrawType;
import maspack.util.InternalErrorException;

/**
 * @author John E Lloyd and ArtiSynth team members
 */
public class GL2Viewer extends GLViewer implements Renderer, HasProperties {

   //   public enum AxialView {
   //      POS_X_POS_Z, NEG_X_POS_Z, POS_X_POS_Y, 
   //      POS_X_NEG_Y, POS_Y_POS_Z, NEG_Y_POS_Z
   //   }

   protected static boolean myUseGLSelectSelection = false;

   private GL2 gl;
   private GL2Resources myGLResources;

   // More control over blending
   public static enum BlendType {
      GL_ONE_MINUS_CONSTANT_ALPHA(GL2.GL_ONE_MINUS_CONSTANT_ALPHA),
      GL_ONE_MINUS_SRC_ALPHA(GL2.GL_ONE_MINUS_SRC_ALPHA),
      GL_ONE(GL2.GL_ONE),
      GL_ZERO(GL2.GL_ZERO),
      GL_SRC_ALPHA(GL2.GL_SRC_ALPHA),
      ;

      private int myValue;
      private BlendType(int val) {
         myValue = val;
      }
      public int value() {
         return myValue;
      }
   }

   public static BlendType DEFAULT_S_BLENDING = BlendType.GL_SRC_ALPHA;
   public static BlendType DEFAULT_D_BLENDING =
   BlendType.GL_ONE_MINUS_CONSTANT_ALPHA;

   private BlendType sBlending = DEFAULT_S_BLENDING;
   private BlendType dBlending = DEFAULT_D_BLENDING;

   private RigidTransform3d Xtmp = new RigidTransform3d();
   private Vector3d utmp = new Vector3d();
   private Vector3d vtmp = new Vector3d();
   private float[] ctmp = new float[3];
   // buffers to store certain line styles
   double[] cosBuff = {1, 0, -1, 0, 1};
   double[] sinBuff = {0, 1, 0, -1, 0};
   private static double[] GLMatrix = new double[16];

   protected boolean myMultiSampleEnabled = false;  // for antialiasing during screenshot
   private GLFrameCapture frameCapture = null;
   private boolean grab = false;
   private boolean grabWaitComplete = false; // wait
   private boolean grabClose = false;

   private boolean rendering2d;

   // Lighting parameters
   protected float lmodel_ambient[] = { 0.0f, 0.0f, 0.0f, 0.0f };
   protected float lmodel_twoside[] = { 0.0f, 0.0f, 0.0f, 0.0f };
   protected float lmodel_local[] = { 0.0f, 0.0f, 0.0f, 0.0f };

   // Color history
   protected float[] myCurrentColor;
   protected Material myCurrentFrontMaterial;
   protected Material myCurrentBackMaterial;
   //overrides diffuse color in myCurrentMaterial:
   protected float[] myCurrentFrontDiffuse;
   protected float[] myCurrentBackDiffuse;

   public static PropertyList myProps = new PropertyList (GL2Viewer.class, GLViewer.class);

   static {
      myProps.add(
         "sBlending", "source transparency blending", DEFAULT_S_BLENDING);
      myProps.add(
         "dBlending", "destination transparency blending", DEFAULT_D_BLENDING);
   }

   public PropertyList getAllPropertyInfo() {
      return myProps;
   }

   private float[] scalefv(float[] c, float s, float[] out) {
      out[0] = c[0]*s;
      out[1] = c[1]*s;
      out[2] = c[2]*s;
      out[3] = c[3]*s;
      return out;
   }
   
   private void setupLight (GL2 gl, GLLight light, float intensityScale) {

      int lightId = light.getId() + GL2.GL_LIGHT0;
      if (light.isEnabled()) {
         gl.glEnable(lightId);
      } else {
         gl.glDisable(lightId);
      }

      if (light.getLightSpace() == LightSpace.CAMERA) {
         gl.glPushMatrix();
         gl.glLoadIdentity();
      }

      float[] tmp = new float[4]; 
      if (light.getType() == LightType.DIRECTIONAL) {
         float[] dir = light.getDirection();
         // negate, since OpenGL expects direction *to* light
         tmp[0] = -dir[0];
         tmp[1] = -dir[1];
         tmp[2] = -dir[2];
         tmp[3] = 0;
         gl.glLightfv (lightId, GL2.GL_POSITION, tmp, 0);
      } else {
         gl.glLightfv (lightId, GL2.GL_POSITION, light.getPosition(), 0);
      }

      gl.glLightfv (lightId, GL2.GL_DIFFUSE, scalefv(light.getDiffuse(),intensityScale,tmp), 0);
      gl.glLightfv (lightId, GL2.GL_AMBIENT, scalefv(light.getAmbient(),intensityScale,tmp), 0);
      gl.glLightfv (lightId, GL2.GL_SPECULAR, scalefv(light.getSpecular(),intensityScale,tmp), 0);

      switch (light.getType()) {
         case DIRECTIONAL:
         case POINT:
            gl.glLighti(lightId, GL2.GL_SPOT_CUTOFF, 180); // special value disabling spot
            break;
         case SPOT:
            gl.glLightf(lightId, GL2.GL_SPOT_CUTOFF, (float)Math.acos(light.getSpotCosCutoff()));
            gl.glLightf(lightId, GL2.GL_SPOT_EXPONENT, light.getSpotExponent());
            gl.glLightfv(lightId, GL2.GL_SPOT_DIRECTION, light.getDirection(), 0);
            break;
      }
      gl.glLightf(lightId, GL2.GL_CONSTANT_ATTENUATION, light.getConstantAttenuation());
      gl.glLightf(lightId, GL2.GL_LINEAR_ATTENUATION, light.getLinearAttenuation());
      gl.glLightf(lightId, GL2.GL_QUADRATIC_ATTENUATION, light.getQuadraticAttenuation());

      if (light.getLightSpace() == LightSpace.CAMERA) {
         gl.glPopMatrix();
      }
   }

   protected int getMaxLights() {
      if (gl != null) {
         return 8;
      } else {
         int[] buff = new int[1];
         gl.glGetIntegerv(GL2.GL_MAX_LIGHTS, buff, 0);
         return buff[0];
      }

   }

   protected void setupLights(GL2 gl) {

      // make sure in modelview matrix mode
      gl.glMatrixMode(GL2.GL_MODELVIEW);

      int maxLights = lightManager.maxLights();
      float intensityScale = 1.0f/lightManager.getMaxIntensity();
      // only enable up to maxLights
      for (GLLight light : lightManager.getLights()) {
         if (light.getId() < maxLights) {
            setupLight(gl, light, intensityScale);
         }
      }
   }

   public void setPointSize(float s) {
      gl.glPointSize(s);
   }

   public float getPointSize() {
      float[] buff = new float[1];
      gl.glGetFloatv(GL.GL_POINT_SIZE, buff, 0);
      return buff[0];
   }

   public void setLineWidth(float w) {
      gl.glLineWidth(w);
   }

   public float getLineWidth() {
      float[] buff = new float[1];
      gl.glGetFloatv(GL.GL_LINE_WIDTH, buff, 0);
      return buff[0];
   }

   /**
    * Performs a selection operation on a sub-region of the viewport.
    * 
    * @param x
    * x coordinate of the selection region center
    * @param y
    * y coordinate of the selection region center
    * @param w
    * selection region width
    * @param h
    * selection region height
    * @param ignoreDepthTest
    * select all objects in the pick frustum, not just those which are
    * visible through the viewport
    */
   public void setPick (
      double x, double y, double w, double h,
      boolean ignoreDepthTest) {

      if (myUseGLSelectSelection) {
         mySelector = new GLSelectSelector (this);
      }
      else if (ignoreDepthTest) {
         mySelector = new GLOcclusionSelector(this);
      }
      else {
         mySelector = new GLColorSelector(this);
      }
      mySelector.setRectangle (x, y, w, h);
      selectTrigger = true;
      repaint();
   }

   public GL getGL() {
      return drawable.getGL().getGL2();
   }

   // end of the rotation code

   /**
    * Creates a new GLViewer with default capabilities.
    * 
    * @param width
    * initial width of the viewer
    * @param height
    * initial height of the viewer
    */
   public GL2Viewer (int width, int height) {
      this (null, null, width, height);
   }

   /**
    * Creates a new GLViewer with default capabilities that shares resources
    * (e.g., diplay lists and textures) with an existing GLViewer.
    * 
    * @param shareWith
    * GLViewer with which resources are to be shared
    * @param width
    * initial width of the viewer
    * @param height
    * initial height of the viewer
    */
   public GL2Viewer (GL2Viewer shareWith, int width, int height) {
      this (null, shareWith.myGLResources, width, height);
   }

   /**
    * Creates a new GLViewer with specified capabilities and size.
    * 
    * @param cap
    * Desired GL capabilities. Can be specified as null, which will create
    * default capabilities.
    * @param shareWith
    * a GL drawable with which the GLCanvas is to share resources (e.g., display
    * lists and textures). Can be specified as null.
    * @param width
    * initial width of the viewer
    * @param height
    * initial height of the viewer
    */
   public GL2Viewer (GLCapabilities cap, GL2Resources sharedResources, int width,
      int height) {

      if (cap == null) {
         GLProfile glp2 = GLProfile.get(GLProfile.GL2);
         cap = new GLCapabilities(glp2);
         cap.setSampleBuffers (true);
         cap.setNumSamples (8);
         
      }

      // canvas = new GLCanvas(cap, sharedContext); //GLCanvas (cap, null, sharedContext, null);
      canvas = new GLCanvas(cap, null, null);
      if (sharedResources != null) {
         canvas.setSharedContext(sharedResources.getContext());
         myGLResources = sharedResources;
      } else {
         myGLResources = new GL2Resources(canvas.getContext());
      }
      canvas.addGLEventListener (this);
      canvas.setPreferredSize(new Dimension(width, height));
      canvas.setSize (width, height);
      
      this.width = width;
      this.height = height;
      myDraggers = new LinkedList<Dragger3d>();
      myUserDraggers = new LinkedList<Dragger3d>();

      lightManager = new GLLightManager();
      setDefaultLights();

      myGrid = new GLGridPlane();
      myGrid.setViewer (this);

      RigidTransform3d EyeToWorld = new RigidTransform3d (0, -3, 0, 1, 0, 0, Math.PI / 2);
      setEyeToWorld(EyeToWorld);
      setAxialView (myAxialView);

      myMouseHandler = new GLMouseAdapter (this);

      if (canvas != null) {
         // canvas.addMouseListener(new GLMouseListener());
         canvas.addMouseListener (myMouseHandler);
         canvas.addMouseWheelListener (myMouseHandler);
         canvas.addMouseMotionListener (myMouseHandler);
         for (MouseInputListener l : myMouseInputListeners) {
            canvas.addMouseListener (l);
            canvas.addMouseMotionListener (l);
         }
         for (MouseWheelListener l : myMouseWheelListeners) {
            canvas.addMouseWheelListener (l);
         }
      }

      buildInternalRenderList();
   }

   private void setDefaultLights() {

      float light0_ambient[] = { 0.1f, 0.1f, 0.1f, 1f };
      float light0_diffuse[] = { 1.0f, 1.0f, 1.0f, 1.0f };
      float light0_position[] = { -0.8660254f, 0.5f, 1f, 0f };
      float light0_specular[] = { 0.5f, 0.5f, 0.5f, 1.0f };

      float light1_ambient[] = { 0.0f, 0.0f, 0.0f, 1.0f };
      float light1_diffuse[] = { 0.5f, 0.5f, 0.5f, 1.0f };
      float light1_position[] = { 0.8660254f, 0.5f, 1f, 0f };
      float light1_specular[] = { 0.5f, 0.5f, 0.5f, 1.0f };

      float light2_ambient[] = { 0.0f, 0.0f, 0.0f, 1.0f };
      float light2_diffuse[] = { 0.5f, 0.5f, 0.5f, 1.0f };
      float light2_position[] = { 0f, -10f, 1f, 0f };
      float light2_specular[] = { 0.5f, 0.5f, 0.5f, 1.0f };

      lightManager.clearLights();
      lightManager.addLight(new GLLight (
         light0_position, light0_ambient, light0_diffuse, light0_specular));
      lightManager.addLight (new GLLight (
         light1_position, light1_ambient, light1_diffuse, light1_specular));
      lightManager.addLight(new GLLight (
         light2_position, light2_ambient, light2_diffuse, light2_specular));
      lightManager.setMaxIntensity(1.0f);
   }

   @Override
   public void init(GLAutoDrawable drawable) {

      fireViewerListenersPreinit(drawable);

      this.drawable = drawable;
      this.gl = drawable.getGL().getGL2();

      gl.setSwapInterval (1);

      if (gl.isExtensionAvailable("GL_ARB_multisample")) {
         gl.glEnable(GL2.GL_MULTISAMPLE);
         myMultiSampleEnabled = true;
      }

      int[] buff = new int[1];
      gl.glGetIntegerv(GL2.GL_MAX_CLIP_PLANES, buff, 0);
      maxClipPlanes = buff[0];

      gl.glEnable (GL2.GL_CULL_FACE);
      gl.glCullFace (GL2.GL_BACK);
      gl.glEnable (GL2.GL_DEPTH_TEST);
      gl.glClearDepth (1.0);

      gl.glLightModelfv (GL2.GL_LIGHT_MODEL_LOCAL_VIEWER, lmodel_local, 0);
      gl.glLightModelfv (GL2.GL_LIGHT_MODEL_TWO_SIDE, lmodel_twoside, 0);
      gl.glLightModelfv (GL2.GL_LIGHT_MODEL_AMBIENT, lmodel_ambient, 0);
      gl.glLightModelf (GL2.GL_LIGHT_MODEL_TWO_SIDE, 1);
      gl.glEnable (GL2.GL_LIGHTING);

      setLightingEnabled(true);
      setDepthEnabled(true);
      setColorEnabled(true);
      setVertexColoringEnabled(true);
      setTextureMappingEnabled(true);
      setFaceMode(Faces.FRONT);
      setShadeModel(Shading.PHONG);
      setGammaCorrectionEnabled(false);

      lightManager.setMaxLights(getMaxLights());
      setupLights(gl);

      gl.glShadeModel (GL2.GL_FLAT);

      // gl.glFrontFace (GL2.GL_CW);
      if (!isSelecting()) {
         gl.glClearColor (bgColor[0], bgColor[1], bgColor[2], bgColor[3]);
      }

      // initialize viewport
      resetViewVolume();
      invalidateModelMatrix();
      invalidateProjectionMatrix();
      invalidateViewMatrix();

      myGLResources.init(gl);

      // trigger rebuild of renderables
      buildInternalRenderList();

      fireViewerListenersPostinit(drawable);
      System.out.println("GL2 initialized");
   }

   @Override
   public void dispose(GLAutoDrawable drawable) {

      fireViewerListenersPredispose(drawable);

      myGLResources.clearCached(gl);

      // nullify stuff
      this.drawable = null;
      this.gl = null;

      fireViewerListenersPostdispose(drawable);
      System.out.println("GL2 disposed");

   }

   public void resetViewVolume(int width, int height) {

      gl.glMatrixMode (GL2.GL_PROJECTION);
      gl.glLoadIdentity();

      double aspect = width / (double)height;
      if (myFrustum.orthographic) {
         double hh = myFrustum.fieldHeight / 2;
         gl.glOrtho (-aspect * hh, aspect * hh, -hh, hh, myFrustum.near, myFrustum.far);
      }
      else {
         if (myFrustum.explicit) {
            gl.glFrustum (myFrustum.left, myFrustum.right, myFrustum.bottom, 
               myFrustum.top, myFrustum.near, myFrustum.far);
         }
         else {
            myFrustum.left = -aspect * myFrustum.top;
            myFrustum.right = -myFrustum.left;
            gl.glFrustum (myFrustum.left, myFrustum.right, myFrustum.bottom,
               myFrustum.top, myFrustum.near, myFrustum.far);
            // glu.gluPerspective(verticalFieldOfView, width/(float)height,
            // myNear, myFar);
         }
      }
      updateProjectionMatrix();
      projectionMatrixValidP = true;  // since we set it here

      gl.glMatrixMode (GL2.GL_MODELVIEW);
      setViewport(0, 0, width, height);
   }

   public void setViewport(int x, int y, int width, int height) {
      gl.glViewport (x, y, width, height);
   }

   public int[] getViewport() {
      int[] buff = new int[4];
      gl.glGetIntegerv(GL.GL_VIEWPORT, buff, 0);
      return buff;
   }

   public void setViewVolume (double near, double far) {
      double aspect = width / (double)height;
      if (myFrustum.orthographic) {
         double hh = myFrustum.fieldHeight / 2;
         gl.glOrtho (-aspect * hh, aspect * hh, -hh, hh, near, far);
      }
      else {
         if (myFrustum.explicit) {
            gl.glFrustum (myFrustum.left, myFrustum.right, myFrustum.bottom, myFrustum.top, near, far);
         }
         else {
            myFrustum.left = -aspect * myFrustum.top;
            myFrustum.right = -myFrustum.left;
            gl.glFrustum (myFrustum.left, myFrustum.right, myFrustum.bottom, myFrustum.top, near, far);
         }
      }

      updateProjectionMatrix();
      projectionMatrixValidP = true;  // since we set it here

   }

   protected void resetViewVolume() {
      gl.glMatrixMode (GL2.GL_PROJECTION);
      gl.glLoadIdentity();
      setViewVolume (myFrustum.near, myFrustum.far);
      gl.glMatrixMode (GL2.GL_MODELVIEW);
      setViewport(0, 0, width, height);
   }

   protected void drawDragBox (GLAutoDrawable drawable) {
      gl.glMatrixMode (GL2.GL_PROJECTION);
      gl.glPushMatrix();
      gl.glLoadIdentity();
      gl.glMatrixMode (GL2.GL_MODELVIEW);
      gl.glPushMatrix();
      gl.glLoadIdentity();

      gl.glDisable (GL2.GL_LIGHTING);
      gl.glColor3f (0.5f, 0.5f, 0.5f);

      double x0 = 2 * myDragBox.x / (double)width - 1;
      double x1 = x0 + 2 * myDragBox.width / (double)width;
      double y0 = 1 - 2 * myDragBox.y / (double)height;
      double y1 = y0 - 2 * myDragBox.height / (double)height;

      gl.glBegin (GL2.GL_LINE_LOOP);
      gl.glVertex3d (x0, y0, 0);
      gl.glVertex3d (x1, y0, 0);
      gl.glVertex3d (x1, y1, 0);
      gl.glVertex3d (x0, y1, 0);
      gl.glEnd();

      gl.glEnable (GL2.GL_LIGHTING);

      gl.glPopMatrix();
      gl.glMatrixMode (GL2.GL_PROJECTION);
      gl.glPopMatrix();
      gl.glMatrixMode (GL2.GL_MODELVIEW);
   }

   public void display (GLAutoDrawable drawable, int flags) {

      if (!myInternalRenderListValid) {
         buildInternalRenderList();
      }

      maybeUpdateMatrices(gl);

      gl.glPushMatrix();
      if (selectTrigger) {
         mySelector.setupSelection (drawable);
         selectEnabled = true;  // moved until after selection initialization
         selectTrigger = false;
      }

      // turn off buffer swapping when doing a selection render because
      // otherwise the previous buffer sometimes gets displayed
      drawable.setAutoSwapBufferMode (selectEnabled ? false : true);

      doDisplay (drawable, flags);

      if (selectEnabled) {
         selectEnabled = false;
         mySelector.processSelection (drawable);
      }
      else {
         fireRerenderListeners();
      }
      gl.glPopMatrix();

      if (frameCapture != null) {
         synchronized(frameCapture) {
            if (grab) {
               offscreenCapture (flags);
               grab = false;
            }
            if (grabWaitComplete) {
               frameCapture.waitForCompletion();
               // reset
               grabWaitComplete = false;
            }
            if (grabClose) {
               frameCapture.waitForCompletion();
               frameCapture.dispose(gl);
               frameCapture = null;
            }
         }
      }
   }

   public boolean isMultiSampleEnabled() {
      return myMultiSampleEnabled;
   }

   private void offscreenCapture (int flags) {

      // Initialize the OpenGL context FOR THE FBO
      gl.setSwapInterval (1);

      // Set rendering commands to go to offscreen frame buffer
      frameCapture.activateFBO(gl);

      // Draw the scene into pbuffer
      gl.glPushMatrix();
      if (selectEnabled) {
         mySelector.setupSelection (drawable);
      }

      resetViewVolume = false;   //disable resetting view volume
      doDisplay (drawable, flags);

      if (selectEnabled) {
         mySelector.processSelection (drawable);
         selectEnabled = false;
      }
      else {
         fireRerenderListeners();
      }
      gl.glPopMatrix();

      // further drawing will go to screen
      frameCapture.deactivateFBO(gl);

      frameCapture.capture(gl);
   }

   private void drawAxes (GL2 gl, double length) {
      double l = length;

      setLightingEnabled (false);

      // draw axis

      gl.glDepthFunc (GL2.GL_ALWAYS);

      if (!selectEnabled) {
         gl.glColor3f (1, 0, 0);
      }
      gl.glBegin (GL2.GL_LINES);
      gl.glVertex3d (l, 0.0, 0.0);
      gl.glVertex3d (0, 0.0, 0.0);
      gl.glEnd();

      if (!selectEnabled) {
         gl.glColor3f (0, 1, 0);
      }
      gl.glBegin (GL2.GL_LINES);
      gl.glVertex3d (0, l, 0.0);
      gl.glVertex3d (0, 0, 0.0);
      gl.glEnd();

      if (!selectEnabled) {
         gl.glColor3f (0, 0, 1);
      }
      gl.glBegin (GL2.GL_LINES);
      gl.glVertex3d (0, 0, l);
      gl.glVertex3d (0, 0, 0);
      gl.glEnd();

      gl.glDepthFunc (GL2.GL_LESS);

      setLightingEnabled (true);
   }

   public void setLightingEnabled(boolean set) {
      if (!selectEnabled) {
         super.setLightingEnabled(set);
         if (set) {
            gl.glEnable(GL2.GL_LIGHTING);
         } else {
            gl.glDisable(GL2.GL_LIGHTING);
         }
      }
   }

   public boolean isLightingEnabled() {
      return gl.glIsEnabled (GL2.GL_LIGHTING);
   }

   private void enableTransparency (GL2 gl) {
      gl.glEnable (GL2.GL_BLEND);
      if (!alphaFaceCulling) {
         gl.glDepthMask (false);
         gl.glDisable (GL2.GL_CULL_FACE);
      }
      gl.glBlendFunc (sBlending.value(), dBlending.value());
   }

   private void disableTransparency (GL2 gl) {
      if (!alphaFaceCulling) {
         gl.glEnable (GL2.GL_CULL_FACE);
         gl.glDepthMask (true);
      }
      gl.glDisable (GL2.GL_BLEND);
   }

   public void setTransparencyEnabled (boolean enable) {

      // do not enable if in selection mode
      if (!(isSelecting() && enable)) {
         if (enable != myTransparencyEnabledP) {
            GL2 gl = drawable.getGL().getGL2();
            if (enable) {
               enableTransparency (gl);
            }
            else {
               disableTransparency (gl);
            }
            myTransparencyEnabledP = enable;
         }
      }
   }               

   public void doDisplay (GLAutoDrawable drawable, int flags) {
      GL2 gl = drawable.getGL().getGL2();

      // updates projection matrix
      if (resetViewVolume && resizeEnabled) {
         resetViewVolume();
         resetViewVolume = false;
      }

      gl.glPushMatrix();

      if (isSelecting()) {
         gl.glClearColor (0f, 0f, 0f, 0f);  
      }
      else {
         gl.glClearColor (bgColor[0], bgColor[1], bgColor[2], bgColor[3]);
      }

      gl.glClear (GL2.GL_COLOR_BUFFER_BIT | GL2.GL_DEPTH_BUFFER_BIT);

      // updates view matrix
      //      RigidTransform3d X = new RigidTransform3d();
      //      X.invert (XEyeToWorld);

      // enter view matrix
      // GLSupport.transformToGLMatrix (GLMatrix, X);
      GLSupport.transformToGLMatrix (GLMatrix, viewMatrix);
      //XXX gl.glMultMatrixd (GLMatrix, 0);  Shouldn't this be load?
      gl.glLoadMatrixd(GLMatrix, 0);
      viewMatrixValidP = true;  // view matrix now "committed"

      setupLights(gl);

      if (!isSelecting()) {
         if (gridVisible) {
            myGrid.render (this, flags);
         }
         if (axisLength > 0) {
            drawAxes (gl, axisLength);
         }

         // rendering dragger separately here so that they are
         // not clipped by the clipping plane
         for (Dragger3d dragger : myDraggers) {
            dragger.render (this, 0);
         }
         if (myDrawTool != null) {
            myDrawTool.render (this, 0);
         }

         for (GLClipPlane cp : myClipPlanes) {
            cp.render (this, flags);
         }
      }

      maybeUpdateMatrices(gl);

      // enable clip planes
      int nclips = 0;
      int clipIdx = GL2.GL_CLIP_PLANE0;
      for (GLClipPlane cp : myClipPlanes) {
         if (cp.isClippingEnabled()) {
            cp.getPlaneValues (myClipPlaneValues );
            myClipPlaneValues[3] += cp.getOffset();
            gl.glClipPlane (clipIdx, myClipPlaneValues, 0);
            gl.glEnable (clipIdx);
            clipIdx++; nclips++;
            if (nclips >= maxClipPlanes) {
               break;
            }

            if (cp.isSlicingEnabled()) {
               myClipPlaneValues[0] = -myClipPlaneValues[0];
               myClipPlaneValues[1] = -myClipPlaneValues[1];
               myClipPlaneValues[2] = -myClipPlaneValues[2];
               myClipPlaneValues[3] =
               -myClipPlaneValues[3]+2*cp.getOffset ();   

               gl.glClipPlane (clipIdx, myClipPlaneValues, 0);
               gl.glEnable (clipIdx);
               clipIdx++;
               nclips++;

               if (nclips >= maxClipPlanes) {
                  break;
               }
            }
         }
      }

      gl.glPushMatrix();

      int qid = 0;
      synchronized(renderablesLock) {
         qid = myInternalRenderList.renderOpaque (this, qid, flags);
         if (myExternalRenderList != null) {
            qid = myExternalRenderList.renderOpaque (this, qid, flags);
         }
      }
      if (!isSelecting()) {
         enableTransparency (gl);
      }

      synchronized(renderablesLock) {
         qid = myInternalRenderList.renderTransparent (this, qid, flags);
         if (myExternalRenderList != null) {
            qid = myExternalRenderList.renderTransparent (this, qid, flags);
         }
      }
      disableTransparency (gl);

      gl.glPopMatrix();

      // disable clipping planes
      for (int i=GL2.GL_CLIP_PLANE0; i<clipIdx; ++i) {
         gl.glDisable(i);
      }

      // Draw 2D objects
      begin2DRendering(width, height);

      synchronized(renderablesLock) {
         qid = myInternalRenderList.renderOpaque2d (this, qid, 0);
         if (myExternalRenderList != null) {
            qid = myExternalRenderList.renderOpaque2d (this, qid, 0);
         }
      }

      enableTransparency (gl);

      synchronized(renderablesLock) {
         qid = myInternalRenderList.renderTransparent2d (this, qid, 0);
         if (myExternalRenderList != null) {
            qid = myExternalRenderList.renderTransparent2d (this, qid, 0);
         }
      }
      disableTransparency (gl);
      end2DRendering();

      gl.glPopMatrix();

      if (!isSelecting()) {
         if (myDragBox != null) {
            drawDragBox (drawable);
         }
      }

      gl.glFlush();
   }

   public static int getNameStackDepth (GL2 gl) {
      int[] depth = new int[1];
      gl.glGetIntegerv (GL2.GL_NAME_STACK_DEPTH, depth, 0);
      return depth[0];
   }

   /**
    * Enable selection via the (now deprecated) OpenGL select buffer
    * and <code>GL_SELECT</code> rendering mode mechanism.
    *
    * @param enable if true, enables select buffer selection
    */
   public static void enableGLSelectSelection (boolean enable) {
      myUseGLSelectSelection = enable;
   }

   public static boolean isGLSelectSelectionEnabled() {
      return myUseGLSelectSelection;
   }


   public void mulTransform (RigidTransform3d X) {
      GLSupport.transformToGLMatrix (GLMatrix, X);
      getGL2().glMultMatrixd (GLMatrix, 0);
   }

   public void mulTransform (AffineTransform3d X) {
      GLSupport.transformToGLMatrix (GLMatrix, X);
      getGL2().glMultMatrixd (GLMatrix, 0);
   }

   //   public void getModelViewMatrix (Matrix4d X) {
   //      getGL2().glGetDoublev (GL2.GL_MODELVIEW_MATRIX, GLMatrix, 0);
   //      for (int i = 0; i < 4; i++) {
   //         for (int j = 0; j < 4; j++) {
   //            X.set (i, j, GLMatrix[j * 4 + i]);
   //         }
   //      }
   //   }
   //
   //   public void getProjectionMatrix (Matrix4d X) {
   //      getGL2().glGetDoublev (GL2.GL_PROJECTION_MATRIX, GLMatrix, 0);
   //      for (int i = 0; i < 4; i++) {
   //         for (int j = 0; j < 4; j++) {
   //            X.set (i, j, GLMatrix[j * 4 + i]);
   //         }
   //      }
   //   }
   //
   //   public void setModelViewMatrix (Matrix4d X) {
   //      getGL2().glMatrixMode (GL2.GL_MODELVIEW); // paranoid
   //      for (int i = 0; i < 4; i++) {
   //         for (int j = 0; j < 4; j++) {
   //            GLMatrix[j * 4 + i] = X.get (i, j);
   //         }
   //      }
   //      getGL2().glLoadMatrixd (GLMatrix, 0);
   //   }
   //   public void setProjectionMatrix (Matrix4d X) {
   //      gl.glMatrixMode (GL2.GL_PROJECTION);
   //      for (int i = 0; i < 4; i++) {
   //         for (int j = 0; j < 4; j++) {
   //            GLMatrix[j * 4 + i] = X.get (i, j);
   //         }
   //      }
   //      gl.glLoadMatrixd (GLMatrix, 0);
   //      gl.glMatrixMode (GL2.GL_MODELVIEW);
   //   }

   private void maybeUpdateMatrices(GL2 gl) {

      if (!viewMatrixValidP || !modelMatrixValidP) {
         // create modelview matrix:
         AffineTransform3d mvmatrix = new AffineTransform3d();
         mvmatrix.mul(viewMatrix, modelMatrix);

         // update modelview matrix
         int[] mmode = new int[1]; 
         gl.glGetIntegerv(GL2.GL_MATRIX_MODE, mmode, 0);
         GLSupport.transformToGLMatrix(GLMatrix, mvmatrix);


         gl.glMatrixMode(GL2.GL_MODELVIEW);
         gl.glLoadMatrixd(GLMatrix,0);

         viewMatrixValidP = true;
         modelMatrixValidP = true;

         gl.glMatrixMode(mmode[0]); // revert
      }

      if (!projectionMatrixValidP) {

         // update projection matrix
         int[] mmode = new int[1]; 
         gl.glGetIntegerv(GL2.GL_MATRIX_MODE, mmode, 0);
         GLSupport.transformToGLMatrix(GLMatrix, projectionMatrix);

         gl.glMatrixMode(GL2.GL_PROJECTION);
         gl.glLoadMatrixd(GLMatrix,0);

         projectionMatrixValidP = true;

         gl.glMatrixMode(mmode[0]);
      }

   }

   @Override
   public boolean popProjectionMatrix() {
      boolean success = super.popProjectionMatrix();
      maybeUpdateMatrices(gl);
      return success;
   }

   @Override
   public void setPickMatrix(float x, float y, float deltax, float deltay, int[] viewport) {
      super.setPickMatrix(x, y, deltax, deltay, viewport);
      maybeUpdateMatrices(gl);
   }

   @Override
   public void clearPickMatrix() {
      super.clearPickMatrix();
      maybeUpdateMatrices(gl);
   }

   public static void mulTransform (GL2 gl, RigidTransform3d X) {
      GLSupport.transformToGLMatrix (GLMatrix, X);
      gl.glMultMatrixd (GLMatrix, 0);
   }

   public static void mulTransform (GL2 gl, AffineTransform3d X) {
      GLSupport.transformToGLMatrix (GLMatrix, X);
      gl.glMultMatrixd (GLMatrix, 0);
   }

   public boolean isSelecting() {
      return selectEnabled;
   }

   @Override
   public void forceColor(float r, float g, float b, float a) {
      float[] rgba = new float[]{r,g,b,a};
      myCurrentColor = rgba;
      gl.glColor4fv (rgba, 0);
   }

   public void setColor (float[] frontRgba, float[] backRgba, boolean selected) {
      if (!selectEnabled) {
         if (selected && myHighlighting == SelectionHighlighting.Color) {
            myCurrentColor = mySelectedColor;
         }
         else if (frontRgba != null) {
            myCurrentColor = frontRgba;
         }
         if (myCurrentColor.length == 3) {
            gl.glColor3fv (myCurrentColor, 0);
         }
         else {
            gl.glColor4fv (myCurrentColor, 0);
         }
      }
   }

   public void setColor (float[] frontRgba, float[] backRgba) {
      if (!selectEnabled) {
         if (frontRgba != null) {
            myCurrentColor = frontRgba;
         }
         if (myCurrentColor.length == 3) {
            gl.glColor3fv (myCurrentColor, 0);
         }
         else {
            gl.glColor4fv (myCurrentColor, 0);
         }
      }
   }

   public void updateColor (float[] frontRgba, float[] backRgba, boolean selected) {
      if (!selectEnabled) {
         float[] c;
         if (selected && myHighlighting == SelectionHighlighting.Color) {
            c = mySelectedColor;
         }
         else {
            c = frontRgba;
         }
         if (myCurrentColor != c) {
            myCurrentColor = c;
            if (myCurrentColor.length == 3) {
               gl.glColor3fv (myCurrentColor, 0);
            }
            else {
               gl.glColor4fv (myCurrentColor, 0);
            }
         }
      }
   }

   public void setMaterial (
      Material frontMaterial, float[] frontDiffuse,
      Material backMaterial, float[] backDiffuse, boolean selected) {
      if (selected && myHighlighting == SelectionHighlighting.Color) {
         myCurrentFrontMaterial = mySelectedMaterial;
         myCurrentBackMaterial = mySelectedMaterial;
         myCurrentFrontDiffuse = null;
         myCurrentBackDiffuse = null;
         myCurrentFrontMaterial.apply (gl, GL.GL_FRONT_AND_BACK);
      }
      else {
         myCurrentFrontMaterial = frontMaterial;
         myCurrentFrontDiffuse = frontDiffuse;
         myCurrentBackMaterial = backMaterial;
         myCurrentBackDiffuse = backDiffuse;
         myCurrentFrontMaterial.apply (gl, GL.GL_FRONT, frontDiffuse);
         myCurrentBackMaterial.apply (gl, GL.GL_BACK, backDiffuse);
      }
   }

   public void setMaterialAndShading (
      RenderProps props, Material frontMaterial, float[] frontDiffuse,
      Material backMaterial, float[] backDiffuse, boolean selected) {

      if (selectEnabled) {
         return;
      }
      Shading shading = props.getShading();
      if (shading == Shading.NONE) {
         setLightingEnabled (false);
         myCurrentColor = null; // ensure color gets set in updateMaterial
         updateMaterial (props, frontMaterial, frontDiffuse, backMaterial, backDiffuse, selected);
      }
      else {
         if (shading != Shading.FLAT) {
            gl.glShadeModel (GL2.GL_SMOOTH);
         }
         myCurrentFrontMaterial = null; // ensure material gets set in updateMaterial
         myCurrentBackMaterial = null;
         myCurrentFrontDiffuse = null;
         myCurrentBackDiffuse = null;
         updateMaterial (props, frontMaterial, frontDiffuse, 
            backMaterial, backDiffuse, selected);
      }
   }

   public void restoreShading (RenderProps props) {
      if (selectEnabled) {
         return;
      }
      Shading shading = props.getShading();
      if (shading == Shading.NONE) {
         setLightingEnabled (true);
      }
      else if (shading != Shading.FLAT) {
         gl.glShadeModel (GL2.GL_FLAT);
      }      
   }

   public void updateMaterial (
      RenderProps props, Material frontMaterial, float[] frontDiffuse, 
      Material backMaterial, float[] backDiffuse, boolean selected) {

      if (selectEnabled) {
         return;
      }
      if (props.getShading() == Shading.NONE) {
         float[] cf;
         if (selected && myHighlighting == SelectionHighlighting.Color) {
            cf = mySelectedColor;
         }
         else if (frontDiffuse != null) {
            cf = new float[4];
            cf[0] = frontDiffuse[0];
            cf[1] = frontDiffuse[1];
            cf[2] = frontDiffuse[2];
            cf[3] = (float)props.getAlpha();
         }
         else {
            cf = frontMaterial.getDiffuse();
         }
         if (cf != myCurrentColor) {
            myCurrentColor = cf;
            if (myCurrentColor.length == 3) {
               gl.glColor3fv (myCurrentColor, 0);
            }
            else {
               gl.glColor4fv (myCurrentColor, 0);
            }
         }
      }
      else {
         Material mf;
         float[] df;
         Material mb;
         float[] db;
         if (selected && myHighlighting == SelectionHighlighting.Color) {
            mf = mySelectedMaterial;
            df = null;
            mb = mySelectedMaterial;
            db = null;
         }
         else {
            mf = frontMaterial;
            df = frontDiffuse;
            mb = backMaterial;
            db = backDiffuse;
         }
         if (myCurrentFrontMaterial != mf || myCurrentFrontDiffuse != df) {
            myCurrentFrontMaterial = mf;
            myCurrentFrontDiffuse = df;
            myCurrentFrontMaterial.apply (gl, GL.GL_FRONT, df);
         }
         if (myCurrentBackMaterial != mb || myCurrentBackDiffuse != db) {
            myCurrentBackMaterial = mb;
            myCurrentBackDiffuse = db;
            myCurrentBackMaterial.apply (gl, GL.GL_BACK, db);
         }
      }
   }

   public void drawSphere (RenderProps props, float[] coords, double r) {

      GL2 gl = getGL2();
      maybeUpdateMatrices(gl);

      gl.glPushMatrix();
      gl.glTranslatef (coords[0], coords[1], coords[2]);
      gl.glScaled (r, r, r);

      boolean normalizeEnabled = gl.glIsEnabled (GL2.GL_NORMALIZE);
      gl.glEnable (GL2.GL_NORMALIZE);

      int slices = props.getPointSlices();
      int displayList = myGLResources.getPrimitiveManager().getSphereDisplayList(gl, slices, slices/2);
      gl.glCallList(displayList);

      if (!normalizeEnabled) {
         gl.glDisable (GL2.GL_NORMALIZE);
      }
      gl.glPopMatrix();
   }

   private void setTriangle (GL2 gl, float[] v0, float[] v1, float[] v2) {
      float ax = v1[0]-v0[0];
      float ay = v1[1]-v0[1];
      float az = v1[2]-v0[2];
      float bx = v2[0]-v0[0];
      float by = v2[1]-v0[1];
      float bz = v2[2]-v0[2];
      gl.glNormal3f (ay*bz-az*by, az*bx-ax*bz, ax*by-ay*bx);
      gl.glVertex3fv (v0, 0);
      gl.glVertex3fv (v1, 0);
      gl.glVertex3fv (v2, 0);
   }

   private void setQuad (GL2 gl, float[] v0, float[] v1, float[] v2, float[] v3) {
      float ax, ay, az;
      float bx, by, bz;
      float nx, ny, nz;

      ax = v1[0]-v0[0];
      ay = v1[1]-v0[1];
      az = v1[2]-v0[2];
      bx = v2[0]-v0[0];
      by = v2[1]-v0[1];
      bz = v2[2]-v0[2];
      nx = ay*bz-az*by;
      ny = az*bx-ax*bz;
      nz = ax*by-ay*bx;
      ax = v3[0]-v0[0];
      ay = v3[1]-v0[1];
      az = v3[2]-v0[2];
      nx += by*az-bz*ay;
      ny += bz*ax-bx*az;
      nz += bx*ay-by*ax;

      gl.glNormal3f (nx, ny, nz);
      gl.glVertex3fv (v0, 0);
      gl.glVertex3fv (v1, 0);
      gl.glVertex3fv (v2, 0);
      gl.glVertex3fv (v3, 0);
   }

   public void drawHex (
      RenderProps props, double scale,
      float[] v0, float[] v1, float[] v2, float[] v3,
      float[] v4, float[] v5, float[] v6, float[] v7) {

      float cx = (v0[0]+v1[0]+v2[0]+v3[0]+v4[0]+v5[0]+v6[0]+v7[0])/8;
      float cy = (v0[1]+v1[1]+v2[1]+v3[1]+v4[1]+v5[1]+v6[1]+v7[1])/8;
      float cz = (v0[2]+v1[2]+v2[2]+v3[2]+v4[2]+v5[2]+v6[2]+v7[2])/8;

      float s = (float)scale;
      GL2 gl = getGL2();
      maybeUpdateMatrices(gl);

      gl.glPushMatrix();
      gl.glTranslatef (cx*(1-s), cy*(1-s), cz*(1-s));
      gl.glScalef (s, s, s);

      boolean normalizeEnabled = gl.glIsEnabled (GL2.GL_NORMALIZE);
      if (!normalizeEnabled) {
         gl.glEnable (GL2.GL_NORMALIZE);
      }

      gl.glBegin (GL2.GL_QUADS);
      setQuad (gl, v0, v1, v2, v3);
      setQuad (gl, v1, v5, v6, v2);
      setQuad (gl, v5, v4, v7, v6);
      setQuad (gl, v4, v0, v3, v7);
      setQuad (gl, v3, v2, v6, v7);
      setQuad (gl, v0, v4, v5, v1);
      gl.glEnd ();

      if (!normalizeEnabled) {
         gl.glDisable (GL2.GL_NORMALIZE);
      }
      gl.glPopMatrix();
   }

   public void drawWedge (
      RenderProps props, double scale,
      float[] v0, float[] v1, float[] v2,
      float[] v3, float[] v4, float[] v5) {

      float cx = (v0[0]+v1[0]+v2[0]+v3[0]+v4[0]+v5[0])/6;
      float cy = (v0[1]+v1[1]+v2[1]+v3[1]+v4[1]+v5[1])/6;
      float cz = (v0[2]+v1[2]+v2[2]+v3[2]+v4[2]+v5[2])/6;

      float s = (float)scale;
      GL2 gl = getGL2();
      maybeUpdateMatrices(gl);

      gl.glPushMatrix();
      gl.glTranslatef (cx*(1-s), cy*(1-s), cz*(1-s));
      gl.glScalef (s, s, s);

      boolean normalizeEnabled = gl.glIsEnabled (GL2.GL_NORMALIZE);
      if (!normalizeEnabled) {
         gl.glEnable (GL2.GL_NORMALIZE);
      }

      gl.glBegin (GL2.GL_QUADS);
      setQuad (gl, v0, v1, v4, v3);
      setQuad (gl, v1, v2, v5, v4);
      setQuad (gl, v2, v0, v3, v5);
      gl.glEnd ();

      gl.glBegin (GL2.GL_TRIANGLES);
      setTriangle (gl, v0, v2, v1);
      setTriangle (gl, v3, v4, v5);
      gl.glEnd ();

      if (!normalizeEnabled) {
         gl.glDisable (GL2.GL_NORMALIZE);
      }
      gl.glPopMatrix();

   }

   public void drawPyramid (
      RenderProps props, double scale,
      float[] v0, float[] v1, float[] v2,
      float[] v3, float[] v4) {

      float cx = (v0[0]+v1[0]+v2[0]+v3[0]+v4[0])/5;
      float cy = (v0[1]+v1[1]+v2[1]+v3[1]+v4[1])/5;
      float cz = (v0[2]+v1[2]+v2[2]+v3[2]+v4[2])/5;

      float s = (float)scale;
      GL2 gl = getGL2();
      maybeUpdateMatrices(gl);

      gl.glPushMatrix();
      gl.glTranslatef (cx*(1-s), cy*(1-s), cz*(1-s));
      gl.glScalef (s, s, s);

      boolean normalizeEnabled = gl.glIsEnabled (GL2.GL_NORMALIZE);
      if (!normalizeEnabled) {
         gl.glEnable (GL2.GL_NORMALIZE);
      }

      gl.glBegin (GL2.GL_QUADS);
      setQuad (gl, v0, v3, v2, v1);
      gl.glEnd ();

      gl.glBegin (GL2.GL_TRIANGLES);
      setTriangle (gl, v0, v1, v4);
      setTriangle (gl, v1, v2, v4);
      setTriangle (gl, v2, v3, v4);
      setTriangle (gl, v3, v0, v4);
      gl.glEnd ();

      if (!normalizeEnabled) {
         gl.glDisable (GL2.GL_NORMALIZE);
      }
      gl.glPopMatrix();

   }

   public void drawTet (
      RenderProps props, double scale,
      float[] v0, float[] v1, float[] v2, float[] v3) {

      float cx = (v0[0]+v1[0]+v2[0]+v3[0])/4;
      float cy = (v0[1]+v1[1]+v2[1]+v3[1])/4;
      float cz = (v0[2]+v1[2]+v2[2]+v3[2])/4;

      float s = (float)scale;
      GL2 gl = getGL2();
      maybeUpdateMatrices(gl);

      gl.glPushMatrix();
      gl.glTranslatef (cx*(1-s), cy*(1-s), cz*(1-s));
      gl.glScalef (s, s, s);

      boolean normalizeEnabled = gl.glIsEnabled (GL2.GL_NORMALIZE);
      if (!normalizeEnabled) {
         gl.glEnable (GL2.GL_NORMALIZE);
      }

      gl.glBegin (GL2.GL_TRIANGLES);
      setTriangle (gl, v0, v2, v1);
      setTriangle (gl, v2, v3, v1);
      setTriangle (gl, v3, v0, v1);
      setTriangle (gl, v0, v3, v2);
      gl.glEnd ();

      if (!normalizeEnabled) {
         gl.glDisable (GL2.GL_NORMALIZE);
      }
      gl.glPopMatrix();
   }

   public void drawTaperedEllipsoid (
      RenderProps props, float[] coords0, float[] coords1) {
      GL2 gl = getGL2();
      maybeUpdateMatrices(gl);

      utmp.set (coords1[0] - coords0[0], coords1[1] - coords0[1], coords1[2]
      - coords0[2]);

      Xtmp.p.set (coords0[0], coords0[1], coords0[2]);
      Xtmp.R.setZDirection (utmp);
      gl.glPushMatrix();
      GL2Viewer.mulTransform (gl, Xtmp);
      double len = utmp.norm();

      gl.glScaled (props.getLineRadius(), props.getLineRadius(), len);

      boolean normalizeEnabled = gl.glIsEnabled (GL2.GL_NORMALIZE);
      gl.glEnable (GL2.GL_NORMALIZE);

      int slices = props.getLineSlices();
      int displayList = myGLResources.getPrimitiveManager().getTaperedEllipsoidDisplayList(gl, slices, slices/2);
      gl.glCallList(displayList);

      if (!normalizeEnabled) {
         gl.glDisable (GL2.GL_NORMALIZE);
      }
      gl.glPopMatrix();
   }

   public void drawSolidArrow (
      RenderProps props, float[] coords0, float[] coords1, boolean capped) {
      //GL2 gl = getGL().getGL2();

      utmp.set (
         coords1[0]-coords0[0], coords1[1]-coords0[1], coords1[2]-coords0[2]);
      double len = utmp.norm();
      utmp.normalize();

      double arrowRad = 3*props.getLineRadius();
      double arrowLen = Math.min(2*arrowRad,len/2);

      ctmp[0] = coords1[0] - (float)(arrowLen*utmp.x);
      ctmp[1] = coords1[1] - (float)(arrowLen*utmp.y);
      ctmp[2] = coords1[2] - (float)(arrowLen*utmp.z);

      drawCylinder (props, coords0, ctmp, capped);
      drawCylinder (props, ctmp, coords1, /*capped=*/true, arrowRad, 0.0);
   }

   public void drawCylinder (
      RenderProps props, float[] coords0, float[] coords1, boolean capped) {
      double r = props.getLineRadius();
      drawCylinder (props, coords0, coords1, capped, r, r);
   }

   public void drawCylinder (
      RenderProps props, float[] coords0, float[] coords1, double r, boolean capped) {
      drawCylinder(props, coords0, coords1, capped, r, r);
   }

   public void drawCylinder (
      RenderProps props, float[] coords0, float[] coords1, boolean capped,
      double base, double top) {

      GL2 gl = getGL2();
      maybeUpdateMatrices(gl);

      // drawing manually like this is 10x faster that gluCylinder, but has
      // no texture coordinates
      int nslices = props.getLineSlices();
      
      drawCylinder(gl,  nslices, (float)base, (float)top, coords0, coords1, capped);

   }

   private void drawCylinder(GL2 gl, int nslices, float base, float top,
      float[] coords0, float[] coords1, boolean capped) {
      
      utmp.set (coords1[0] - coords0[0], coords1[1] - coords0[1], coords1[2]
      - coords0[2]);
      Xtmp.p.set (coords0[0], coords0[1], coords0[2]);
      Xtmp.R.setZDirection (utmp);
      gl.glPushMatrix();
      GL2Viewer.mulTransform (gl, Xtmp);

      double h = utmp.norm();

      // fill angle buffer
      if (nslices+1 != cosBuff.length) {
         cosBuff = new double[nslices+1];
         sinBuff = new double[nslices+1];
         cosBuff[0] = 1;
         sinBuff[0] = 0;
         cosBuff[nslices] = 1;
         sinBuff[nslices] = 0;
         for (int i=1; i<nslices; i++) {
            double ang = i / (double)nslices * 2 * Math.PI;
            cosBuff[i] = Math.cos(ang);
            sinBuff[i] = Math.sin(ang);
         }
      }
      
      double nz = (base-top)/h;
      double nscale = 1.0/Math.sqrt(1+nz*nz);

      // draw sides
      gl.glBegin(GL2.GL_QUAD_STRIP);
      double c1,s1;
      for (int i = 0; i <= nslices; i++) {
         c1 = cosBuff[i];
         s1 = sinBuff[i];
         gl.glNormal3d(nscale*c1, nscale*s1, nscale*nz);
         gl.glVertex3d (top * c1, top * s1, h);
         gl.glVertex3d (base * c1, base * s1, 0);
      }

      gl.glEnd();

      if (capped) { // draw top cap first
         if (top > 0) {
            gl.glBegin (GL2.GL_POLYGON);
            gl.glNormal3d (0, 0, 1);
            for (int i = 0; i < nslices; i++) {
               gl.glVertex3d (top * cosBuff[i], top * sinBuff[i], h);
            }
            gl.glEnd();
         }
         // now draw bottom cap
         if (base > 0) {
            gl.glBegin (GL2.GL_POLYGON);
            gl.glNormal3d (0, 0, -1);
            for (int i = nslices-1; i >=0; i--) {
               gl.glVertex3d (base * cosBuff[i], base * sinBuff[i], 0);
            }
            gl.glEnd();
         }
      }
      gl.glPopMatrix();
   }

   @Override
   public void drawCone(
      RenderProps props, float[] coords0, float[] coords1, double r,
      boolean capped) {
      drawCylinder(props, coords0, coords1, capped, r, 0);
   }

   public void drawLine (
      RenderProps props, float[] coords0, float[] coords1, boolean capped,
      float[] color, boolean selected) {

      GL2 gl = getGL2();
      maybeUpdateMatrices(gl);

      switch (props.getLineStyle()) {
         case LINE: {
            setLightingEnabled (false);
            gl.glLineWidth (props.getLineWidth());
            if (color == null) {
               color = props.getLineColorArray ();
            }
            if (props.getAlpha () < 1) {
               color = new float[]{color[0], color[1], color[2], (float)props.getAlpha ()};
            }
            setColor (color, selected);
            gl.glBegin (GL2.GL_LINES);
            gl.glVertex3fv (coords0, 0);
            gl.glVertex3fv (coords1, 0);
            gl.glEnd();
            gl.glLineWidth (1);
            setLightingEnabled (true);
            break;
         }
         case CYLINDER: {
            setMaterialAndShading (props, props.getLineMaterial(), color, selected);
            drawCylinder (props, coords0, coords1, capped);
            restoreShading (props);
            break;
         }
         case SOLID_ARROW: {
            setMaterialAndShading (props, props.getLineMaterial(), color, selected);
            drawSolidArrow (props, coords0, coords1, capped);
            restoreShading (props);
            break;
         }
         case ELLIPSOID: {
            setMaterialAndShading (props, props.getLineMaterial(), color, selected);
            drawTaperedEllipsoid (props, coords0, coords1);
            restoreShading (props);
            break;
         }
         default: {
            throw new InternalErrorException (
               "Unimplemented line style " + props.getLineStyle());
         }
      }
   }

   public void drawArrow (
      RenderProps props, float[] coords0, float[] coords1, boolean capped,
      boolean selected) {

      GL2 gl = getGL2();
      maybeUpdateMatrices(gl);

      utmp.set (coords1[0] - coords0[0], coords1[1] - coords0[1], coords1[2]
      - coords0[2]);
      double len = utmp.norm();

      utmp.normalize();
      vtmp.set (coords1[0], coords1[1], coords1[2]);
      double arrowRad = 3 * props.getLineRadius();
      double arrowLen = 2*arrowRad;
      vtmp.scaledAdd (-len + arrowLen, utmp);
      ctmp[0] = (float)vtmp.x;
      ctmp[1] = (float)vtmp.y;
      ctmp[2] = (float)vtmp.z;

      switch (props.getLineStyle()) {
         case LINE: {
            setMaterialAndShading (props, props.getLineMaterial(), selected);
            if (len <= arrowLen) {
               drawCylinder (props, coords1, coords0, capped, len, 0.0);
            }
            else {
               setLightingEnabled (false);
               gl.glLineWidth (props.getLineWidth());
               setColor (props.getLineColorArray(), selected);
               gl.glBegin (GL2.GL_LINES);
               gl.glVertex3fv (coords1, 0);
               gl.glVertex3fv (ctmp, 0);
               gl.glEnd();
               gl.glLineWidth (1);
               setLightingEnabled (true);
               drawCylinder (
                  props, ctmp, coords0, capped, arrowRad, 0.0);
            }
            restoreShading (props);
            break;
         }
         case CYLINDER: {
            setMaterialAndShading (props, props.getLineMaterial(), selected);
            if (len <= arrowLen) {
               drawCylinder (props, coords1, coords0, capped, len, 0.0);
            }
            else {
               drawCylinder (props, ctmp, coords1, capped);
               drawCylinder (
                  props, ctmp, coords0, capped, arrowRad, 0.0);
            }
            restoreShading (props);
            break;
         }
         case ELLIPSOID: {
            setMaterialAndShading (props, props.getLineMaterial(), selected);
            if (len <= arrowLen) {
               drawCylinder (props, coords1, coords0, capped, len, 0.0);
            }
            else {
               drawTaperedEllipsoid (props, coords0, coords1);
               drawCylinder (
                  props, ctmp, coords0, capped, arrowRad, 0.0);
            }
            restoreShading (props);
            break;
         }
         default: {
            throw new InternalErrorException (
               "Unimplemented line style " + props.getLineStyle());
         }
      }
   }

   public void drawPoint (RenderProps props, float[] coords, boolean selected) {

      GL2 gl = getGL2();
      maybeUpdateMatrices(gl);

      switch (props.getPointStyle()) {
         case POINT: {
            int size = props.getPointSize();
            if (size > 0) {
               setLightingEnabled (false);
               gl.glPointSize (size);
               setColor (props.getPointColorArray(), selected);
               gl.glBegin (GL2.GL_POINTS);
               gl.glVertex3fv (coords, 0);
               gl.glEnd();
               gl.glPointSize (1);
               setLightingEnabled (true);
            }
            break;
         }
         case SPHERE: {
            setMaterialAndShading (props, props.getPointMaterial(), selected);
            drawSphere (props, coords);
            restoreShading (props);
            break;
         }
      }
   }

   public void drawPoint (float[] coords) {

      GL2 gl = getGL2();
      maybeUpdateMatrices(gl);

      gl.glBegin (GL2.GL_POINTS);
      gl.glVertex3fv (coords, 0);
      gl.glEnd();

   }

   @Override
   public void drawLine(float[] coords0, float[] coords1) {

      GL2 gl = getGL2();
      maybeUpdateMatrices(gl);

      gl.glBegin (GL2.GL_LINES);
      gl.glVertex3fv (coords0, 0);
      gl.glVertex3fv (coords1, 0);
      gl.glEnd();

   }

   @Override
   public void drawPoints (
      RenderProps props, Iterator<? extends RenderablePoint> iterator) {

      GL2 gl = getGL2();
      maybeUpdateMatrices(gl);

      switch (props.getPointStyle()) {
         case POINT: {
            int size = props.getPointSize();
            if (size > 0) {
               // draw regular points first
               setLightingEnabled (false);
               gl.glPointSize (size);
               if (isSelecting()) {
                  // don't worry about color in selection mode
                  int i = 0;
                  while (iterator.hasNext()) {
                     RenderablePoint pnt = iterator.next();
                     if (pnt.getRenderProps() == null) {
                        if (isSelectable (pnt)) {
                           beginSelectionQuery (i);
                           gl.glBegin (GL2.GL_POINTS);
                           gl.glVertex3fv (pnt.getRenderCoords(), 0);
                           gl.glEnd();
                           endSelectionQuery ();
                        }
                     }
                     i++;
                  }
               }
               else {
                  gl.glBegin (GL2.GL_POINTS);
                  setColor (props.getPointColorArray(), false);
                  while (iterator.hasNext()) {
                     RenderablePoint pnt = iterator.next();
                     if (pnt.getRenderProps() == null) {
                        updateColor (
                           props.getPointColorArray(), pnt.isSelected());
                        gl.glVertex3fv (pnt.getRenderCoords(), 0);
                     }
                  }
                  gl.glEnd();
               }
               gl.glPointSize (1);
               setLightingEnabled (true);
            }
            break;
         }
         case SPHERE: {
            setMaterialAndShading (props, props.getPointMaterial(), false);
            int i = 0;
            while (iterator.hasNext()) {
               RenderablePoint pnt = iterator.next();
               if (pnt.getRenderProps() == null) {
                  if (isSelecting()) {
                     if (isSelectable (pnt)) {
                        beginSelectionQuery (i);
                        drawSphere (props, pnt.getRenderCoords());
                        endSelectionQuery ();      
                     }
                  }
                  else {
                     updateMaterial (
                        props, props.getPointMaterial(), pnt.isSelected());
                     drawSphere (props, pnt.getRenderCoords());
                  }
               }
               i++;
            }
            restoreShading (props);
         }
      }
   }

   public static void drawLineStrip (
      Renderer renderer, Iterable<float[]> vertexList, RenderProps props,
      LineStyle lineStyle, boolean isSelected) {

      if ( !(renderer instanceof GL2Viewer) ){
         return;
      }

      GL2Viewer viewer = (GL2Viewer)renderer;

      GL2 gl = viewer.getGL2();
      viewer.maybeUpdateMatrices(gl);

      switch (lineStyle) {
         case LINE: {
            viewer.setLightingEnabled (false);
            // draw regular points first
            gl.glLineWidth (props.getLineWidth());
            gl.glBegin (GL2.GL_LINE_STRIP);
            viewer.setColor (props.getLineColorArray(), isSelected);
            for (float[] v : vertexList) {
               gl.glVertex3fv (v, 0);
            }
            gl.glEnd();
            gl.glLineWidth (1);
            viewer.setLightingEnabled (true);
            break;
         }
         case ELLIPSOID:
         case SOLID_ARROW:
         case CYLINDER: {
            viewer.setMaterialAndShading (
               props, props.getLineMaterial(), isSelected);
            float[] v0 = null;
            for (float[] v1 : vertexList) {
               if (v0 != null) {
                  if (lineStyle == LineStyle.ELLIPSOID) {
                     viewer.drawTaperedEllipsoid (props, v0, v1);
                  }
                  else if (lineStyle == LineStyle.SOLID_ARROW) {
                     viewer.drawSolidArrow (props, v0, v1);
                  }
                  else {
                     viewer.drawCylinder (props, v0, v1);
                  }
               }
               else {
                  v0 = new float[3];
               }
               v0[0] = v1[0];
               v0[1] = v1[1];
               v0[2] = v1[2];
            }
            viewer.restoreShading (props);
         }
      }
   }

   public void drawLines (
      RenderProps props, Iterator<? extends RenderableLine> iterator) {

      GL2 gl = getGL2();
      maybeUpdateMatrices(gl);

      LineStyle lineStyle = props.getLineStyle();
      switch (lineStyle) {
         case LINE: {
            setLightingEnabled (false);
            // draw regular points first
            gl.glLineWidth (props.getLineWidth());
            if (isSelecting()) {
               // don't worry about color in selection mode
               int i = 0;
               while (iterator.hasNext()) {
                  RenderableLine line = iterator.next();
                  if (line.getRenderProps() == null) {
                     if (isSelectable (line)) {
                        beginSelectionQuery (i);
                        gl.glBegin (GL2.GL_LINES);
                        gl.glVertex3fv (line.getRenderCoords0(), 0);
                        gl.glVertex3fv (line.getRenderCoords1(), 0);
                        gl.glEnd();
                        endSelectionQuery ();
                     }
                  }
                  i++;
               }
            }
            else {
               gl.glBegin (GL2.GL_LINES);
               setColor (props.getLineColorArray(), false);
               while (iterator.hasNext()) {
                  RenderableLine line = iterator.next();
                  if (line.getRenderProps() == null) {
                     if (line.getRenderColor() == null) {
                        updateColor (props.getLineColorArray(),line.isSelected());
                     }
                     else {
                        updateColor (line.getRenderColor(),line.isSelected());
                     }
                     gl.glVertex3fv (line.getRenderCoords0(), 0);
                     gl.glVertex3fv (line.getRenderCoords1(), 0);
                  }
               }
               gl.glEnd();
            }
            gl.glLineWidth (1);
            setLightingEnabled (true);
            break;
         }
         case ELLIPSOID:
         case SOLID_ARROW:
         case CYLINDER: {
            // GLU glu = getGLU();
            setMaterialAndShading (
               props, props.getLineMaterial(), /*selected=*/false);
            int i = 0;
            while (iterator.hasNext()) {
               RenderableLine line = iterator.next();
               float[] v0 = line.getRenderCoords0();
               float[] v1 = line.getRenderCoords1();
               if (line.getRenderProps() == null) {
                  if (isSelecting()) {
                     if (isSelectable (line)) {
                        beginSelectionQuery (i);
                        if (lineStyle == LineStyle.ELLIPSOID) {
                           drawTaperedEllipsoid (props, v0, v1);
                        }
                        else if (lineStyle == LineStyle.SOLID_ARROW) {
                           drawSolidArrow (props, v0, v1, /*capped=*/true);
                        }
                        else {
                           drawCylinder (props, v0, v1);
                        }
                        endSelectionQuery ();
                     }
                  }
                  else {
                     maspack.render.Material mat = props.getLineMaterial();
                     updateMaterial (
                        props, mat, line.getRenderColor(), line.isSelected());
                     if (lineStyle == LineStyle.ELLIPSOID) {
                        drawTaperedEllipsoid (props, v0, v1);
                     }
                     else if (lineStyle == LineStyle.SOLID_ARROW) {
                        drawSolidArrow (props, v0, v1, /*capped=*/true);
                     }
                     else {
                        drawCylinder (props, v0, v1);
                     }
                  }
               }
               i++;
            }
            restoreShading (props);
            break;
         }
      }
   }

   public void drawAxes (
      RenderProps props, RigidTransform3d X, double len, boolean selected) {

      GL2 gl = getGL2();
      maybeUpdateMatrices(gl);

      if (X == null) {
         X = RigidTransform3d.IDENTITY;
      }

      Vector3d u = new Vector3d();
      setLightingEnabled (false);
      gl.glLineWidth (props.getLineWidth());
      if (selected) {
         setColor (null, selected);
      }
      gl.glBegin (GL2.GL_LINES);
      for (int i = 0; i < 3; i++) {
         if (!selected && !selectEnabled) {
            gl.glColor3f (i == 0 ? 1f : 0f, i == 1 ? 1f : 0f, i == 2 ? 1f : 0f);
         }
         gl.glVertex3d (X.p.x, X.p.y, X.p.z);
         X.R.getColumn (i, u);
         gl.glVertex3d (X.p.x + len * u.x, X.p.y + len * u.y, X.p.z + len * u.z);
      }
      gl.glEnd();
      gl.glLineWidth (1);
      setLightingEnabled (true);
   }

   public void drawAxes (
      RenderProps props, RigidTransform3d X, double [] len, boolean selected) {

      GL2 gl = getGL2();
      maybeUpdateMatrices(gl);

      Vector3d u = new Vector3d();
      setLightingEnabled (false);
      gl.glLineWidth (props.getLineWidth());

      if (X == null) {
         X = RigidTransform3d.IDENTITY;
      }

      if (selected) {
         setColor (null, selected);
      }

      gl.glBegin (GL2.GL_LINES);
      for (int i = 0; i < 3; i++) {
         if (len[i] != 0) {
            if (!selected && !selectEnabled) {
               gl.glColor3f (
                  i == 0 ? 1f : 0f, i == 1 ? 1f : 0f, i == 2 ? 1f : 0f);
            }
            gl.glVertex3d (X.p.x, X.p.y, X.p.z);
            X.R.getColumn (i, u);
            gl.glVertex3d (
               X.p.x + len[i] * u.x, X.p.y + len[i] * u.y, X.p.z + len[i] * u.z);
         }
      }

      gl.glEnd();
      gl.glLineWidth (1);
      setLightingEnabled (true);
   }

   @Override
   public void drawLines(float[] vertices, int flags) {
      maybeUpdateMatrices(gl);
      gl.glBegin(GL2.GL_LINES);
      for (int i=0; i<vertices.length/3; i++) {
         gl.glVertex3fv(vertices, 3*i);
      }
      gl.glEnd();

   }

   public void setFaceMode (RenderProps.Faces mode) {
      switch (mode) {
         case FRONT_AND_BACK: {
            gl.glDisable (GL2.GL_CULL_FACE);
            break;
         }
         case FRONT: {
            gl.glEnable (GL2.GL_CULL_FACE);
            gl.glCullFace (GL2.GL_BACK);
            break;
         }
         case BACK: {
            gl.glEnable (GL2.GL_CULL_FACE);
            gl.glCullFace (GL2.GL_FRONT);
            break;
         }
         case NONE: {
            gl.glEnable (GL2.GL_CULL_FACE);
            gl.glCullFace (GL2.GL_FRONT_AND_BACK);
            break;
         }
      }
   }

   public void setDefaultFaceMode() {
      gl.glEnable (GL2.GL_CULL_FACE);
      gl.glCullFace (GL2.GL_BACK);
   }

   //   private int glClipPlaneToIndex (int glClipPlane) {
   //      return glClipPlane - GL2.GL_CLIP_PLANE0;
   //   }

   private int indexToGlClipPlane (int index) {
      if (index > 5) {
         throw new InternalErrorException (
            "No clip plane for index " + index);
      }
      return GL2.GL_CLIP_PLANE0+index;
   }

   /**
    * Setup for a screenshot during the next render cycle
    * @param w width of shot
    * @param h height of shot
    * @param samples number of samples to use for the
    *        multisample FBO (does antialiasing)
    * @param file
    * @param format
    */
   public void setupScreenShot (
      int w, int h, int samples, File file, String format) {

      if (frameCapture == null) {
         frameCapture = new GLFrameCapture ( w, h, samples, file, format);
      }
      else {
         synchronized(frameCapture) {
            frameCapture.reconfigure(w, h, samples, isGammaCorrectionEnabled(), file, format);
         }
      }
      grab = true;
   }

   public void setupScreenShot (
      int w, int h, File file, String format) {
      setupScreenShot(w, h, -1, file, format);
   }

   public void awaitScreenShotCompletion() {
      if (frameCapture != null) {
         grabWaitComplete = true;  // signal to wait after next grab
         repaint();                // execute in render thread
         // frameCapture.waitForCompletion();
      }
   }

   public boolean grabPending() {
      return grab;
   }

   public void cleanupScreenShots () {
      grabClose = true;
      repaint();  // execute in render thread
   }

   public BlendType getSBlending() {
      return sBlending;
   }

   public void setSBlending(BlendType glBlendValue) {
      sBlending = glBlendValue;
   }

   public BlendType getDBlending() {
      return dBlending;
   }

   public void setDBlending(BlendType glBlendValue) {
      dBlending = glBlendValue;
   }

   @Override
   public void begin2DRendering(double left, double right, double bottom, double top) {

      int attribBits = 
      GL2.GL_ENABLE_BIT | GL2.GL_TEXTURE_BIT | GL2.GL_COLOR_BUFFER_BIT 
      |GL2.GL_DEPTH_BUFFER_BIT | GL2.GL_TRANSFORM_BIT;
      gl.glPushAttrib(attribBits);

      setLightingEnabled (false);
      gl.glDisable(GL2.GL_DEPTH_TEST);
      gl.glDisable(GL2.GL_CULL_FACE);

      // XXX modify global projection, model and view matrices?
      gl.glMatrixMode(GL2.GL_TEXTURE);
      gl.glPushMatrix();
      gl.glLoadIdentity();
      gl.glMatrixMode(GL2.GL_PROJECTION);
      gl.glPushMatrix();
      gl.glLoadIdentity();
      gl.glOrtho(left, right, bottom, top, -1, 1);
      gl.glMatrixMode(GL2.GL_MODELVIEW);
      gl.glPushMatrix();
      gl.glLoadIdentity();
      rendering2d = true;

   }

   @Override
   public void end2DRendering() {

      gl.glMatrixMode(GL2.GL_TEXTURE);
      gl.glPopMatrix();
      gl.glMatrixMode(GL2.GL_PROJECTION);
      gl.glPopMatrix();
      gl.glMatrixMode(GL2.GL_MODELVIEW);
      gl.glPopMatrix();
      gl.glPopAttrib();
      rendering2d = false;
   }

   @Override
   public boolean is2DRendering() {
      return rendering2d;
   }

   @Override
   public GL2 getGL2() {
      return drawable.getGL().getGL2();
   }

   @Override
   public void drawTriangles(RenderObject robj) {
      maybeUpdateMatrices(gl);

      List<int[]> tris = robj.getTriangles();

      if (tris != null) {

         boolean normalizeEnabled = gl.glIsEnabled (GL2.GL_NORMALIZE);
         gl.glEnable (GL2.GL_NORMALIZE);

         boolean enableLighting = false;
         if (isLightingEnabled() && !robj.hasNormals()) {
            enableLighting = true;
            setLightingEnabled(false);
         }

         boolean selecting = isSelecting();
         boolean useColors = (robj.hasColors() && isVertexColoringEnabled());
         boolean useDisplayList = !selecting || !useColors;
         DisplayListPassport dlpp = null;
         RenderObjectKey key = new RenderObjectKey(robj, DrawType.TRIANGLES);
         RenderObjectVersion fingerprint = robj.getVersionInfo();
         boolean compile = true;

         if (useDisplayList) {
            dlpp = getDisplayListPassport(gl, key);
            if (dlpp == null) {
               dlpp = allocateDisplayListPassport(gl, key, fingerprint);
               compile = true;
            } else {
               compile = !(dlpp.compareExchangeFingerPrint(fingerprint));
            }
         }

         if (compile) {
            if (dlpp != null) {
               gl.glNewList(dlpp.getList(), GL2.GL_COMPILE);
            }

            gl.glBegin(GL.GL_TRIANGLES);

            for (int[] tri : tris) {
               for (int i=0; i<3; ++i) {
                  VertexIndexSet v = robj.getVertex(tri[i]);
                  if (!selecting && useColors) {
                     gl.glColor4ubv(robj.getColor(v.getColorIndex()), 0);
                  }
                  if (robj.hasNormals()) {
                     gl.glNormal3fv(robj.getNormal(v.getNormalIndex()), 0);
                  }
                  gl.glVertex3fv(robj.getPosition(v.getPositionIndex()), 0);
               }
            }

            gl.glEnd();

            if (dlpp != null) {
               gl.glEndList();
               gl.glCallList(dlpp.getList());
            }
         } else {
            gl.glCallList(dlpp.getList());
         }


         if (enableLighting) {
            setLightingEnabled(true);
         }

         if (!normalizeEnabled) {
            gl.glDisable (GL2.GL_NORMALIZE);
         }
      }

   }

   private static class PointFingerPrint {

      private PointStyle style;
      private int sphereDL;
      private float r;
      private RenderObjectVersion rv;
      
      public PointFingerPrint(RenderObjectVersion rv, PointStyle style, int sphereDL, float r) {
         this.rv = rv;
         this.style = style;
         this.sphereDL = sphereDL;
         this.r = r;
      }

      @Override
      public int hashCode() {
         return (style == null ? 0 : style.ordinal()) + 71*sphereDL + Float.hashCode(r)*51 + 31*rv.hashCode();
      }

      @Override
      public boolean equals(Object obj) {
         if (obj == this) {
            return true;
         }
         if (obj == null || obj.getClass() != this.getClass()) {
            return false;
         }
         PointFingerPrint other = (PointFingerPrint)obj;
         if (style != other.style || sphereDL != other.sphereDL || r != other.r) {
            return false;
         }
         return rv.equals(other.rv);
      }

   }

   private static class LineFingerPrint {

      private RenderObjectVersion rv;
      private LineStyle style;
      private int slices;
      private float r;
      public LineFingerPrint(RenderObjectVersion rv, LineStyle style, int slices, float r) {
         this.rv = rv;
         this.style = style;
         this.slices = slices;
         this.r = r;
      }

      @Override
      public int hashCode() {
         return (style == null ? 0 : style.ordinal()) + 71*slices + Float.hashCode(r)*51 + 31*rv.hashCode();
      }

      @Override
      public boolean equals(Object obj) {
         if (obj == this) {
            return true;
         }
         if (obj == null || obj.getClass() != this.getClass()) {
            return false;
         }
         LineFingerPrint other = (LineFingerPrint)obj;
         if (style != other.style || slices != other.slices || r != other.r) {
            return false;
         }
         return rv.equals(other.rv);
      }

   }

   private static class VertexFingerPrint {
      
      private RenderObjectVersion rv;
      private VertexDrawMode mode;
      
      public VertexFingerPrint(RenderObjectVersion rv,VertexDrawMode mode) {
         this.rv = rv;
         this.mode = mode;
      }

      @Override
      public int hashCode() {
         return (mode == null ? 0 : mode.ordinal()) + 31*rv.hashCode();
      }

      @Override
      public boolean equals(Object obj) {
         if (obj == this) {
            return true;
         }
         if (obj == null || obj.getClass() != this.getClass()) {
            return false;
         }
         VertexFingerPrint other = (VertexFingerPrint)obj;
         if (mode != other.mode) {
            return false;
         }
         return rv.equals(other.rv);
      }

   }
   
   @Override
   public void drawLines(RenderObject robj) {
      maybeUpdateMatrices(gl);
      List<int[]> lines = robj.getLines();

      if (lines != null) {
         boolean normalizeEnabled = gl.glIsEnabled (GL2.GL_NORMALIZE);
         gl.glEnable (GL2.GL_NORMALIZE);

         boolean enableLighting = false;
         if (isLightingEnabled() && !robj.hasNormals()) {
            enableLighting = true;
            setLightingEnabled(false);
         }

         boolean selecting = isSelecting();
         boolean useColors = (robj.hasColors() && isVertexColoringEnabled());
         boolean useDisplayList = !selecting || !useColors;
         DisplayListPassport dlpp = null;
         RenderObjectKey key = new RenderObjectKey(robj, DrawType.LINES);
         LineFingerPrint fingerprint = new LineFingerPrint(robj.getVersionInfo(), LineStyle.LINE, 0, 0);
         boolean compile = true;

         if (useDisplayList) {
            dlpp = getDisplayListPassport(gl, key);
            if (dlpp == null) {
               dlpp = allocateDisplayListPassport(gl, key, fingerprint);
               compile = true;
            } else {
               compile = !(dlpp.compareExchangeFingerPrint(fingerprint));
            }
         }

         if (compile) {
            if (dlpp != null) {
               gl.glNewList(dlpp.getList(), GL2.GL_COMPILE);
            }

            gl.glBegin(GL.GL_LINES);

            for (int[] line : lines) {
               for (int i=0; i<2; ++i) {
                  VertexIndexSet v = robj.getVertex(line[i]);
                  if (!selecting && useColors) {
                     gl.glColor4ubv(robj.getColor(v.getColorIndex()), 0);
                  }
                  if (robj.hasNormals()) {
                     gl.glNormal3fv(robj.getNormal(v.getNormalIndex()), 0);
                  }
                  gl.glVertex3fv(robj.getPosition(v.getPositionIndex()), 0);
               }
            }

            gl.glEnd();
            if (dlpp != null) {
               gl.glEndList();
               gl.glCallList(dlpp.getList());
            }
         } else {
            gl.glCallList(dlpp.getList());
         }

         if (enableLighting) {
            setLightingEnabled(true);
         }

         if (!normalizeEnabled) {
            gl.glDisable (GL2.GL_NORMALIZE);
         }
      }
   }

   private void drawColoredCylinder (GL2 gl, int nslices, double base,
      double top, float[] coords0, byte[] color0, float[] coords1, 
      byte[] color1, boolean capped) {

      utmp.set (coords1[0] - coords0[0], coords1[1] - coords0[1], coords1[2]
      - coords0[2]);
      Xtmp.p.set (coords0[0], coords0[1], coords0[2]);
      Xtmp.R.setZDirection (utmp);

      gl.glPushMatrix();
      GL2Viewer.mulTransform (gl, Xtmp);

      double h = utmp.norm();

      // maybe re-fill angle buffer
      if (nslices+1 != cosBuff.length) {
         cosBuff = new double[nslices+1];
         sinBuff = new double[nslices+1];
         cosBuff[0] = 1;
         sinBuff[0] = 0;
         cosBuff[nslices] = 1;
         sinBuff[nslices] = 0;
         for (int i=1; i<nslices; i++) {
            double ang = i / (double)nslices * 2 * Math.PI;
            cosBuff[i] = Math.cos(ang);
            sinBuff[i] = Math.sin(ang);
         }
      }

      // draw sides
      gl.glBegin(GL2.GL_QUAD_STRIP);
      double c1,s1;
      for (int i = 0; i <= nslices; i++) {
         c1 = cosBuff[i];
         s1 = sinBuff[i];
         gl.glNormal3d(c1, s1, (base-top)/h);
         gl.glColor4ubv (color0, 0);
         gl.glVertex3d (base * c1, base * s1, 0);

         gl.glColor4ubv (color1, 0);
         gl.glVertex3d (top * c1, top * s1, h);
      }

      gl.glEnd();


      if (capped) { // draw top cap first
         gl.glColor4ubv(color1, 0);
         if (top > 0) {
            gl.glBegin (GL2.GL_POLYGON);
            gl.glNormal3d (0, 0, 1);
            for (int i = 0; i < nslices; i++) {
               gl.glVertex3d (top * cosBuff[i], top * sinBuff[i], h);
            }
            gl.glEnd();
         }
         // now draw bottom cap
         gl.glColor4ubv(color0, 0);
         if (base > 0) {
            gl.glBegin (GL2.GL_POLYGON);
            gl.glNormal3d (0, 0, -1);
            for (int i = 0; i < nslices; i++) {
               gl.glVertex3d (base * cosBuff[i], base * sinBuff[i], 0);
            }
            gl.glEnd();
         }
      }

      gl.glPopMatrix();

   }

   /**
    * Converts hsv to rgb.  It is safe to have the same
    * array as both input and output.
    */
   protected static void HSVtoRGB(byte[] hsv, byte[] rgb ) {
      float h = (float)hsv[0]/255;
      float s = (float)hsv[1]/255;
      float v = (float)hsv[2]/255;

      if (s == 0) {
         rgb[0] = (byte)(v*255);
         rgb[1] = (byte)(v*255);
         rgb[2] = (byte)(v*255);
      } else {
         h = (float)(h - Math.floor(h))* 6;
         float f = h - (float)Math.floor(h);
         float p = v * (1.0f - s);
         float q = v * (1.0f - s * f);
         float t = v * (1.0f - (s * (1.0f - f)));
         switch ((int) h) {
            case 0:
               rgb[0] = (byte)(v*255);
               rgb[1] = (byte)(t*255);
               rgb[2] = (byte)(p*255);
               break;
            case 1:
               rgb[0] = (byte)(q*255); 
               rgb[1] = (byte)(v*255); 
               rgb[2] = (byte)(p*255); 
               break;
            case 2:
               rgb[0] = (byte)(p*255);
               rgb[1] = (byte)(v*255);
               rgb[2] = (byte)(t*255);
               break;
            case 3:
               rgb[0] = (byte)(p*255);
               rgb[1] = (byte)(q*255);
               rgb[2] = (byte)(v*255);
               break;
            case 4:
               rgb[0] = (byte)(t*255);
               rgb[1] = (byte)(p*255);
               rgb[2] = (byte)(v*255);
               break;
            case 5:
               rgb[0] = (byte)(v*255);
               rgb[1] = (byte)(p*255);
               rgb[2] = (byte)(q*255);
               break;
         }
      }

   }

   /**
    * Converts rgb to hsv.  It is safe to have the same
    * array as both input and output.
    */
   protected static void RGBtoHSV(byte[] rgb, byte[] hsv) {
      float r = (float)rgb[0]/255;
      float g = (float)rgb[1]/255;
      float b = (float)rgb[2]/255;

      float cmax = (r > g) ? r : g;
      if (b > cmax) cmax = b;
      float cmin = (r < g) ? r : g;
      if (b < cmin) cmin = b;

      hsv[2] = (byte)(cmax*255);
      if (cmax != 0) {
         hsv[1] = (byte)( (cmax - cmin) / cmax * 255);
      } else {
         hsv[1] = 0;
      }
      if (hsv[1] == 0) {
         hsv[0] = 0;
      } else {
         float hue = 0;
         float redc = (cmax - r) / (cmax - cmin);
         float greenc = (cmax - g) / (cmax - cmin);
         float bluec = (cmax - b) / (cmax - cmin);
         if (r == cmax) {
            hue = bluec - greenc;
         } else if (g == cmax) {
            hue = 2.0f + redc - bluec;
         } else {
            hue = 4.0f + greenc - redc;
         }
         hue = hue / 6.0f;
         if (hue < 0) {
            hue = hue + 1.0f;
         }
         hsv[0] = (byte)(hue*255);
      }
   }

   private void drawArrow (GL2 gl, int nslices, double rad,
      float arrowRad, float arrowHeight,
      float[] coords0,float[] coords1, boolean capped) {

      utmp.set (coords1[0] - coords0[0], coords1[1] - coords0[1], coords1[2]
      - coords0[2]);
      Xtmp.p.set (coords0[0], coords0[1], coords0[2]);
      Xtmp.R.setZDirection (utmp);

      gl.glPushMatrix();
      GL2Viewer.mulTransform (gl, Xtmp);

      double h2 = utmp.norm();
      double h = h2-arrowHeight;


      // maybe re-fill angle buffer
      if (nslices+1 != cosBuff.length) {
         cosBuff = new double[nslices+1];
         sinBuff = new double[nslices+1];
         cosBuff[0] = 1;
         sinBuff[0] = 0;
         cosBuff[nslices] = 1;
         sinBuff[nslices] = 0;
         for (int i=1; i<nslices; i++) {
            double ang = i / (double)nslices * 2 * Math.PI;
            cosBuff[i] = Math.cos(ang);
            sinBuff[i] = Math.sin(ang);
         }
      }

      // draw shaft
      gl.glBegin(GL2.GL_QUAD_STRIP);
      double c1,s1;
      for (int i = 0; i <= nslices; i++) {
         c1 = cosBuff[i];
         s1 = sinBuff[i];
         gl.glNormal3d(c1, s1, 0);
         gl.glVertex3d (rad * c1, rad * s1, h);  
         gl.glVertex3d (rad * c1, rad * s1, 0);
      }
      gl.glEnd();

      // arrow
      gl.glBegin(GL2.GL_QUAD_STRIP);
      for (int i = 0; i <= nslices; i++) {
         c1 = cosBuff[i];
         s1 = sinBuff[i];
         gl.glNormal3d(c1, s1, 1);
         gl.glVertex3d (0, 0, h2);      
         gl.glVertex3d (arrowRad * c1, arrowRad * s1, h);   
      }
      gl.glEnd();

      if (capped) { 
         // bottom cap
         gl.glBegin (GL2.GL_POLYGON);
         gl.glNormal3d (0, 0, -1);
         for (int i = 0; i < nslices; i++) {
            gl.glVertex3d (-rad * cosBuff[i], rad * sinBuff[i], 0);
         }
         gl.glEnd();

         // connection
         gl.glBegin (GL2.GL_QUAD_STRIP);
         gl.glNormal3d (0, 0, -1);
         for (int i = 0; i <= nslices; i++) {
            gl.glVertex3d (arrowRad * cosBuff[i], arrowRad * sinBuff[i], h);
            gl.glVertex3d (rad * cosBuff[i], rad * sinBuff[i], h);
         }
         gl.glEnd();
      }

      gl.glPopMatrix();

   }

   private void drawColoredArrow (GL2 gl, int nslices, double rad,
      float arrowRad, float arrowHeight,
      float[] coords0, byte[] color0, float[] coords1, 
      byte[] color1, boolean hsv, boolean capped) {

      utmp.set (coords1[0] - coords0[0], coords1[1] - coords0[1], coords1[2]
      - coords0[2]);
      Xtmp.p.set (coords0[0], coords0[1], coords0[2]);
      Xtmp.R.setZDirection (utmp);

      gl.glPushMatrix();
      GL2Viewer.mulTransform (gl, Xtmp);

      // interpolate color
      byte[] colorM = new byte[4];
      double h2 = utmp.norm();
      double h = h2-arrowHeight;
      double t = (float)(h/h2);
      interpColor4ub(color0, t, color1, colorM, hsv);

      // maybe re-fill angle buffer
      if (nslices+1 != cosBuff.length) {
         cosBuff = new double[nslices+1];
         sinBuff = new double[nslices+1];
         cosBuff[0] = 1;
         sinBuff[0] = 0;
         cosBuff[nslices] = 1;
         sinBuff[nslices] = 0;
         for (int i=1; i<nslices; i++) {
            double ang = i / (double)nslices * 2 * Math.PI;
            cosBuff[i] = Math.cos(ang);
            sinBuff[i] = Math.sin(ang);
         }
      }

      // draw shaft
      gl.glBegin(GL2.GL_QUAD_STRIP);
      double c1,s1;
      for (int i = 0; i <= nslices; i++) {
         c1 = cosBuff[i];
         s1 = sinBuff[i];
         gl.glNormal3d(c1, s1, 0);

         gl.glColor4ubv (colorM, 0);
         gl.glVertex3d (rad * c1, rad * s1, h);   

         gl.glColor4ubv (color0, 0);
         gl.glVertex3d (rad * c1, rad * s1, 0);      
      }
      gl.glEnd();

      // arrow
      gl.glBegin(GL2.GL_QUAD_STRIP);
      for (int i = 0; i <= nslices; i++) {
         c1 = cosBuff[i];
         s1 = sinBuff[i];
         gl.glNormal3d(c1, s1, 1);

         gl.glColor4ubv (color1, 0);
         gl.glVertex3d (0, 0, h2);   

         gl.glColor4ubv (colorM, 0);
         gl.glVertex3d (rad * c1, rad * s1, h);

      }
      gl.glEnd();

      if (capped) { 
         // bottom cap
         gl.glColor4ubv(color0, 0);
         gl.glBegin (GL2.GL_POLYGON);
         gl.glNormal3d (0, 0, -1);
         for (int i = 0; i < nslices; i++) {
            gl.glVertex3d (rad * cosBuff[i], rad * sinBuff[i], 0);
         }
         gl.glEnd();

         // connection
         gl.glColor4ubv(colorM, 0);
         gl.glBegin (GL2.GL_QUAD_STRIP);
         gl.glNormal3d (0, 0, -1);
         for (int i = 0; i < nslices; i++) {
            gl.glVertex3d (rad * cosBuff[i], rad * sinBuff[i], h);
            gl.glVertex3d (arrowRad * cosBuff[i], arrowRad * sinBuff[i], h);
         }
         gl.glEnd();
      }

      gl.glPopMatrix();

   }

   private void drawEllipsoid(GL2 gl, int slices, float rad, float[] p0, float[] p1) {

      utmp.set (p1[0] - p0[0], p1[1] - p0[1], p1[2]
      - p0[2]);
      Xtmp.p.set (p0[0], p0[1], p0[2]);
      Xtmp.R.setZDirection (utmp);

      gl.glPushMatrix();
      GL2Viewer.mulTransform (gl, Xtmp);

      int levels = slices/2;
      levels = Math.max(levels, 2);

      double s0 = 0;
      double c0 = 1;
      double len = utmp.norm();
      
      // gl.glScaled(rad, rad, len);

      for (int slice = 0; slice < slices; slice++) {
         double ang = (slice + 1) * 2 * Math.PI / slices;
         double c1 = Math.cos (ang);
         double s1 = Math.sin (ang);

         gl.glBegin (GL2.GL_TRIANGLE_STRIP);
         for (int j = 0; j <= levels; j++) {
            double h = j * 1.0 / levels;
            double r = 1 * Math.sin (h * Math.PI);
            double drdh = Math.PI * Math.cos (h * Math.PI);
            gl.glNormal3d (c0, s0, -drdh*rad/len);
            gl.glVertex3d (c0 * r * rad, s0 * r * rad, h * len);
            gl.glNormal3d (c1, s1, -drdh*rad/len);
            gl.glVertex3d (c1 * r * rad, s1 * r * rad, h * len);
         }
         gl.glEnd();

         s0 = s1;
         c0 = c1;
      }

      gl.glPopMatrix();
   }

   private void interpColor4ub(byte[] c0, double t, byte[] c1, byte[] out, boolean hsv) {
      if (hsv) {
         byte[] tmp = new byte[4];
         RGBtoHSV(c0, tmp);
         tmp[3] = c0[3];
         RGBtoHSV(c1, out);
         out[3] = c1[3];
         for (int i=0; i<4; ++i) {
            out[i] = (byte)((1-t)*tmp[i] + t*out[i]);
         }
         HSVtoRGB(out, out);
      } else {
         for (int i=0; i<4; ++i) {
            out[i] = (byte)((1-t)*c0[i] + t*c1[i]);
         }
      }
   }

   private void drawColoredEllipsoid(GL2 gl, int slices, float rad, float[] p0, byte[] c0,
      float[] p1, byte[] c1, boolean hsv) {

      utmp.set (p1[0] - p0[0], p1[1] - p0[1], p1[2]
      - p0[2]);
      Xtmp.p.set (p0[0], p0[1], p0[2]);
      Xtmp.R.setZDirection (utmp);

      gl.glPushMatrix();
      GL2Viewer.mulTransform (gl, Xtmp);

      int levels = slices/2;
      levels = Math.max(levels, 2);

      double sin0 = 0;
      double cos0 = 1;
      double len = utmp.norm();

      // gl.glScaled(rad, rad, len);
      
      byte[] cm = new byte[4];

      for (int slice = 0; slice < slices; slice++) {
         double ang = (slice + 1) * 2 * Math.PI / slices;
         double cos1 = Math.cos (ang);
         double sin1 = Math.sin (ang);

         gl.glBegin (GL2.GL_TRIANGLE_STRIP);
         for (int j = 0; j <= levels; j++) {
            double h = j * 1.0 / levels;
            double r = 1 * Math.sin (h * Math.PI / 1.0);
            double drdh = Math.PI / 1.0 * 1.0 * Math.cos (h * Math.PI / 1.0);

            interpColor4ub(c0, h, c1, cm, hsv);
            gl.glColor4ubv(cm, 0);
            gl.glNormal3d (cos0, sin0, -drdh*rad/len);
            gl.glVertex3d (cos0 * r * rad, sin0 * r * rad, h*len);
            gl.glNormal3d (cos1, sin1, -drdh*rad/len);
            gl.glVertex3d (cos1 * r * rad, sin1 * r * rad, h*len);
         }
         gl.glEnd();

         sin0 = sin1;
         cos0 = cos1;
      }

      gl.glPopMatrix();
   }

   private void drawSolidLines(RenderObject robj, LineStyle style, float rad) {
      List<int[]> lines = robj.getLines();

      boolean normalizeEnabled = gl.glIsEnabled (GL2.GL_NORMALIZE);
      if (!normalizeEnabled) {
         gl.glEnable (GL2.GL_NORMALIZE);
      }
      
      int slices = 64;
      boolean selecting = isSelecting();
      boolean useColors = (robj.hasColors() && isVertexColoringEnabled());
      boolean useDisplayList = !selecting || !useColors;
      DisplayListPassport dlpp = null;
      RenderObjectKey key = new RenderObjectKey(robj, DrawType.LINES);
      LineFingerPrint fingerprint = new LineFingerPrint(robj.getVersionInfo(), style, slices, rad);
      boolean compile = true;

      if (useDisplayList) {
         dlpp = getDisplayListPassport(gl, key);
         if (dlpp == null) {
            dlpp = allocateDisplayListPassport(gl, key, fingerprint);
            compile = true;
         } else {
            compile = !(dlpp.compareExchangeFingerPrint(fingerprint));
         }
      }

      if (compile) {
         if (dlpp != null) {
            gl.glNewList(dlpp.getList(), GL2.GL_COMPILE);
         }

         switch (style) {
            case CYLINDER: {
               if (!selecting && useColors) {
                  for (int[] line : lines) {
                     VertexIndexSet v0 = robj.getVertex(line[0]);
                     VertexIndexSet v1 = robj.getVertex(line[1]);
                     float[] p0 = robj.getPosition(v0.getPositionIndex());
                     byte[] c0 = robj.getColor(v0.getColorIndex());
                     float[] p1 = robj.getPosition(v1.getPositionIndex());
                     byte[] c1 = robj.getColor(v1.getColorIndex());
                     drawColoredCylinder(gl, slices, rad, rad, p0, c0, p1, c1, true);
                  }
               } else {
                  for (int[] line : lines) {
                     VertexIndexSet v0 = robj.getVertex(line[0]);
                     VertexIndexSet v1 = robj.getVertex(line[1]);
                     float[] p0 = robj.getPosition(v0.getPositionIndex());
                     float[] p1 = robj.getPosition(v1.getPositionIndex());
                     drawCylinder(gl, slices, rad, rad, p0, p1, true);
                  }
               }
               break;
            }
            case ELLIPSOID:
               if (!selecting && useColors) {
                  for (int[] line : lines) {
                     VertexIndexSet v0 = robj.getVertex(line[0]);
                     VertexIndexSet v1 = robj.getVertex(line[1]);
                     float[] p0 = robj.getPosition(v0.getPositionIndex());
                     byte[] c0 = robj.getColor(v0.getColorIndex());
                     float[] p1 = robj.getPosition(v1.getPositionIndex());
                     byte[] c1 = robj.getColor(v1.getColorIndex());
                     drawColoredEllipsoid(gl, slices, rad, p0, c0, p1, c1,
                        isHSVColorInterpolationEnabled());
                  }
               } else {
                  for (int[] line : lines) {
                     VertexIndexSet v0 = robj.getVertex(line[0]);
                     VertexIndexSet v1 = robj.getVertex(line[1]);
                     float[] p0 = robj.getPosition(v0.getPositionIndex());
                     float[] p1 = robj.getPosition(v1.getPositionIndex());
                     drawEllipsoid(gl, slices, rad, p0, p1);
                  }
               }
               break;
            case SOLID_ARROW: {
               float arad = rad*3;
               float aheight = arad*2;
               if (!selecting && useColors) {
                  for (int[] line : lines) {
                     VertexIndexSet v0 = robj.getVertex(line[0]);
                     VertexIndexSet v1 = robj.getVertex(line[1]);
                     float[] p0 = robj.getPosition(v0.getPositionIndex());
                     byte[] c0 = robj.getColor(v0.getColorIndex());
                     float[] p1 = robj.getPosition(v1.getPositionIndex());
                     byte[] c1 = robj.getColor(v1.getColorIndex());
                     drawColoredArrow(gl, slices, rad, arad, aheight, p0, c0, p1, c1, 
                        isHSVColorInterpolationEnabled(), true);
                  }
               } else {
                  for (int[] line : lines) {
                     VertexIndexSet v0 = robj.getVertex(line[0]);
                     VertexIndexSet v1 = robj.getVertex(line[1]);
                     float[] p0 = robj.getPosition(v0.getPositionIndex());
                     float[] p1 = robj.getPosition(v1.getPositionIndex());
                     drawArrow(gl, slices, rad, arad, aheight, p0, p1, true);
                  }
               }
               break;
            }
            default:
         }

         if (dlpp != null) {
            gl.glEndList();
            gl.glCallList(dlpp.getList());
         }
      } else {
         gl.glCallList(dlpp.getList());
      }
      
      if (!normalizeEnabled) {
         gl.glDisable (GL2.GL_NORMALIZE);
      }
   }

   @Override
   public void drawLines(RenderObject robj, LineStyle style, double rad) {
      maybeUpdateMatrices(gl);

      switch (style) {
         case LINE: {
            // maybe change line width
            float fold = getLineWidth();
            float frad = (float)rad;
            boolean changeWidth = false;
            if (fold != frad) {
               setLineWidth(frad);
               changeWidth = true;
            }
            drawLines(robj);
            if (changeWidth) {
               setLineWidth(fold);
            }
            break;
         }
         case CYLINDER:
         case ELLIPSOID:
         case SOLID_ARROW:
            drawSolidLines(robj, style, (float)rad);
            break;
      }
   }

   @Override
   public void drawPoints(RenderObject robj) {
      maybeUpdateMatrices(gl);
      List<int[]> pnts = robj.getPoints();

      if (pnts != null) {
         boolean normalizeEnabled = gl.glIsEnabled (GL2.GL_NORMALIZE);
         gl.glEnable (GL2.GL_NORMALIZE);

         boolean enableLighting = false;
         if (isLightingEnabled() && !robj.hasNormals()) {
            enableLighting = true;
            setLightingEnabled(false);
         }

         boolean selecting = isSelecting();
         boolean useColors = (robj.hasColors() && isVertexColoringEnabled());
         boolean useDisplayList = !selecting || !useColors;
         DisplayListPassport dlpp = null;
         RenderObjectKey key = new RenderObjectKey(robj, DrawType.POINTS);
         PointFingerPrint fingerprint = new PointFingerPrint(robj.getVersionInfo(), PointStyle.POINT, 0, 0);
         boolean compile = true;

         if (useDisplayList) {
            dlpp = getDisplayListPassport(gl, key);
            if (dlpp == null) {
               dlpp = allocateDisplayListPassport(gl, key, fingerprint);
               compile = true;
            } else {
               compile = !(dlpp.compareExchangeFingerPrint(fingerprint));
            }
         }

         if (compile) {
            if (dlpp != null) {
               gl.glNewList(dlpp.getList(), GL2.GL_COMPILE);
            }

            gl.glBegin(GL.GL_POINTS);

            for (int[] pnt : pnts) {
               VertexIndexSet v = robj.getVertex(pnt[0]);
               if (!selecting && robj.hasColors() && isVertexColoringEnabled()) {
                  gl.glColor4ubv(robj.getColor(v.getColorIndex()),0);
               }
               if (robj.hasNormals()) {
                  gl.glNormal3fv(robj.getNormal(v.getNormalIndex()),0);
               }
               gl.glVertex3fv(robj.getPosition(v.getPositionIndex()), 0);
            }

            gl.glEnd();

            if (dlpp != null) {
               gl.glEndList();
               gl.glCallList(dlpp.getList());
            }
         } else {
            gl.glCallList(dlpp.getList());
         }

         if (enableLighting) {
            setLightingEnabled(true);
         }
         if (!normalizeEnabled) {
            gl.glDisable (GL2.GL_NORMALIZE);
         }
      }
   }

   private void drawSpheres(RenderObject robj, double rad) {
      GL2 gl = getGL2();
      maybeUpdateMatrices(gl);

      boolean normalizeEnabled = gl.glIsEnabled (GL2.GL_NORMALIZE);
      gl.glEnable (GL2.GL_NORMALIZE);

      List<int[]> pnts = robj.getPoints();

      int slices = 64; // hard-code for now?
      int displayList = myGLResources.getPrimitiveManager().getSphereDisplayList(gl, slices, slices/2);

      boolean selecting = isSelecting();
      boolean useColors = (robj.hasColors() && isVertexColoringEnabled());
      boolean useDisplayList = !selecting || !useColors;
      DisplayListPassport dlpp = null;
      RenderObjectKey key = new RenderObjectKey(robj, DrawType.POINTS);
      PointFingerPrint fingerprint = new PointFingerPrint(robj.getVersionInfo(), PointStyle.SPHERE, displayList, (float)rad);
      boolean compile = true;

      if (useDisplayList) {
         dlpp = getDisplayListPassport(gl, key);
         if (dlpp == null) {
            dlpp = allocateDisplayListPassport(gl, key, fingerprint);
            compile = true;
         } else {
            compile = !(dlpp.compareExchangeFingerPrint(fingerprint));
         }
      }

      if (compile) {
         if (dlpp != null) {
            gl.glNewList(dlpp.getList(), GL2.GL_COMPILE);
         }

         for (int[] pnt : pnts) {
            VertexIndexSet v = robj.getVertex(pnt[0]);
            // maybe color or XXX texture
            if (!selecting && useColors) {
               byte[] c = robj.getColor(v.getColorIndex());
               float[] fc = new float[]{(float)c[0]/255f, (float)c[1]/255f, (float)c[2]/255f, (float)c[3]/255f}; 
               updateColor(fc, false); // XXX update material?
            }
            // position
            float [] p = robj.getPosition(v.getPositionIndex());

            // location and scale
            gl.glPushMatrix();
            gl.glTranslatef (p[0], p[1], p[2]);
            gl.glScaled (rad, rad, rad);   

            // draw sphere
            gl.glCallList(displayList);
            gl.glPopMatrix();

         }

         if (dlpp != null) {
            gl.glEndList();
            gl.glCallList(dlpp.getList());
         }
      } else {
         gl.glCallList(dlpp.getList());
      }

      if (!normalizeEnabled) {
         gl.glDisable (GL2.GL_NORMALIZE);
      }
   }

   @Override
   public void drawPoints(RenderObject robj, PointStyle style, double rad) {
      maybeUpdateMatrices(gl);

      switch (style) { 
         case POINT: {
            // maybe change point size and draw points
            float fold = getPointSize();
            float frad = (float)rad;
            boolean changed = false;
            if (fold != frad) {
               setPointSize(frad);
               changed = true;
            }
            drawPoints(robj);
            if (changed) {
               setPointSize(fold);
            }
            break;
         }
         case SPHERE:
            // draw spheres
            drawSpheres(robj, rad);
            break;
      }

   }

   @Override
   public void drawVertices(RenderObject robj, VertexDrawMode mode) {

      maybeUpdateMatrices(gl);

      boolean normalizeEnabled = gl.glIsEnabled (GL2.GL_NORMALIZE);
      gl.glEnable (GL2.GL_NORMALIZE);

      boolean enableLighting = false;
      if (isLightingEnabled() && !robj.hasNormals()) {
         enableLighting = true;
         setLightingEnabled(false);
      }

      boolean selecting = isSelecting();
      boolean useColors = (robj.hasColors() && isVertexColoringEnabled());
      boolean useDisplayList = !selecting || !useColors;
      DisplayListPassport dlpp = null;
      RenderObjectKey key = new RenderObjectKey(robj, DrawType.VERTICES);
      VertexFingerPrint fingerprint = new VertexFingerPrint(robj.getVersionInfo(), mode);
      boolean compile = true;

      if (useDisplayList) {
         dlpp = getDisplayListPassport(gl, key);
         if (dlpp == null) {
            dlpp = allocateDisplayListPassport(gl, key, fingerprint);
            compile = true;
         } else {
            compile = !(dlpp.compareExchangeFingerPrint(fingerprint));
         }
      }

      if (compile) {
         if (dlpp != null) {
            gl.glNewList(dlpp.getList(), GL2.GL_COMPILE);
         }

         switch (mode) {
            case LINES:
               gl.glBegin(GL2.GL_LINES);
               break;
            case LINE_LOOP:
               gl.glBegin(GL2.GL_LINE_LOOP);
               break;
            case LINE_STRIP:
               gl.glBegin(GL2.GL_LINE_STRIP);
               break;
            case POINTS:
               gl.glBegin(GL2.GL_POINTS);
               break;
            case TRIANGLES:
               gl.glBegin(GL2.GL_TRIANGLES);
               break;
            case TRIANGLE_FAN:
               gl.glBegin(GL2.GL_TRIANGLE_FAN);
               break;
            case TRIANGLE_STRIP:
               gl.glBegin(GL2.GL_TRIANGLE_STRIP);
               break;
            default:
               gl.glBegin(GL2.GL_POINTS);
               break;
         }

         for (VertexIndexSet v : robj.getVertices()) {
            if (!selecting && useColors) {
               gl.glColor4ubv(robj.getColor(v.getColorIndex()),0);
            }
            if (robj.hasNormals()) {
               gl.glNormal3fv(robj.getNormal(v.getNormalIndex()),0);
            }
            gl.glVertex3fv(robj.getPosition(v.getPositionIndex()), 0);
         }


         gl.glEnd();

         if (dlpp != null) {
            gl.glEndList();
            gl.glCallList(dlpp.getList());
         }
      } else {
         gl.glCallList(dlpp.getList());
      }

      if (enableLighting) {
         setLightingEnabled(true);
      }
      if (!normalizeEnabled) {
         gl.glDisable (GL2.GL_NORMALIZE);
      }

   }

   @Override
   public void draw(RenderObject r) {
      drawPoints(r);
      drawLines(r);
      drawTriangles(r);
   }

   @Override
   public RenderObject getSharedObject(Object key) {
      return myGLResources.getRenderObject(key);
   }

   @Override
   public void addSharedObject(Object key, RenderObject r) {
      myGLResources.addRenderObject(key, r);
   }

   @Override
   public void removeSharedObject(Object key) {
      myGLResources.removeRenderObject(key);
   }

   public int getSphereDisplayList(GL2 gl, int slices) {
      GL2PrimitiveManager primManager = myGLResources.getPrimitiveManager();
      int list = primManager.getSphereDisplayList(gl, slices, slices/2);
      return list;
   }

   public int getCylinderDisplayList(GL2 gl, int slices, boolean capped) {
      GL2PrimitiveManager primManager = myGLResources.getPrimitiveManager();
      int list = primManager.getCylinderDisplayList(gl, slices, capped);
      return list;
   }

   public int getTaperedEllipsoidDisplayList(GL2 gl, int slices) {
      GL2PrimitiveManager primManager = myGLResources.getPrimitiveManager();
      int list = primManager.getTaperedEllipsoidDisplayList(gl, slices, slices/2);
      return list;
   }

   public DisplayListPassport getDisplayListPassport(GL2 gl, Object key) {
      DisplayListManager dlMan = myGLResources.getDisplayListManager();
      DisplayListPassport list = dlMan.getDisplayList(gl, key);
      return list;
   }

   public DisplayListPassport allocateDisplayListPassport(GL2 gl, Object key, Object fingerPrint) {
      DisplayListManager dlMan = myGLResources.getDisplayListManager();
      DisplayListPassport list = dlMan.allocateDisplayList(gl, key, fingerPrint);
      return list;
   }

   public int getDisplayList(GL2 gl, Object key) {
      DisplayListManager dlMan = myGLResources.getDisplayListManager();
      DisplayListPassport list = dlMan.getDisplayList(gl, key);
      return list.getList();
   }

   public int allocateDisplayList(GL2 gl, Object key) {
      DisplayListManager dlMan = myGLResources.getDisplayListManager();
      DisplayListPassport list = dlMan.allocateDisplayList(gl, key);
      return list.getList();
   }

   public void freeDisplayList(GL2 gl, Object key) {
      DisplayListManager dlMan = myGLResources.getDisplayListManager();
      dlMan.freeDisplayList(gl, key);
   }

}
