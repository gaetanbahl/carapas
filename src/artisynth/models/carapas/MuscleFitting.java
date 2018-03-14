package artisynth.models.carapas;

import java.awt.Color;
import java.awt.Rectangle;
import java.awt.event.ActionEvent;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.PrintWriter;
import java.io.Reader;

import javax.swing.JButton;
import javax.swing.JFileChooser;
import javax.swing.filechooser.FileNameExtensionFilter;

import maspack.matrix.Matrix3d;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.RotationMatrix3d;

import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.Vector3d;
import maspack.properties.PropertyList;
import maspack.render.RenderProps;
import maspack.render.Renderer;
import maspack.render.Renderer.LineStyle;
import maspack.render.Renderer.PointStyle;
import maspack.util.NumberFormat;
import maspack.util.ReaderTokenizer;

import artisynth.core.driver.Main;
import artisynth.core.femmodels.FemFactory;
import artisynth.core.femmodels.FemFactory.FemElementType;
import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.gui.ControlPanel;
import artisynth.core.gui.editorManager.FrameMarkerAgent;
import artisynth.core.materials.LinearAxialMaterial;
import artisynth.core.materials.LinearMaterial;
import artisynth.core.mechmodels.AxialSpring;
import artisynth.core.mechmodels.Collidable;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.Point;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.workspace.RootModel;

public class MuscleFitting extends RootModel {
   
   MechModel mechMod;
   FemModel3d femMod;
   
   ControlPanel pan;
   
   boolean addBone = false;
   boolean addPoints = false;
   boolean createMuscle = false;
   boolean saveMuscle = false;
   boolean loadMuscle = false;
   boolean reset = false;
   
   static PropertyList simProps = new PropertyList(MuscleFitting.class, RootModel.class);
   
   @Override
   public PropertyList getAllPropertyInfo() {
      return simProps;
   }
   
   static {
      simProps.add ("addBone", "Add Bone", false);
      simProps.add ("addPoints", "Add Points", false);
      simProps.add ("createMuscle", "Create Muscle from FrameMarkers", false);
      simProps.add ("saveMuscle", "Save Muscle", false);
      simProps.add ("loadMuscle", "Load Muscle", false);
      simProps.add ("reset", "Reset", false);
   }
   
   public boolean getAddBone () {
      return addBone;
   }

   public void setAddBone (boolean addBone) {
      this.addBone = false;
      BoneAgent agent = new MuscleFittingBoneAgent(Main.getMain (), mechMod);
      agent.show (new Rectangle(640,480,640,480));
   }

   public boolean getAddPoints () {
      return addPoints;
   }

   public void setAddPoints (boolean addPoints) {
      this.addPoints = false;
      FrameMarkerAgent agent = new FrameMarkerAgent(Main.getMain (), mechMod);
      agent.show (new Rectangle(640,480,640,480));
   }

   public boolean getCreateMuscle () {
      return createMuscle;
   }

   public void setCreateMuscle (boolean createMuscle) {
      this.createMuscle = false;
      createFrame();
   }

   public boolean getSaveMuscle () {
      return saveMuscle;
   }

   public void setSaveMuscle (boolean saveMuscle) {
      this.saveMuscle = false;
      JFileChooser chooser = new JFileChooser();
      FileNameExtensionFilter filter = new FileNameExtensionFilter(
          "OBJ files", "obj");
      chooser.setFileFilter(filter);
      int returnVal = chooser.showOpenDialog(this.getControlPanelTabs ());
      if(returnVal == JFileChooser.APPROVE_OPTION) {
         try {
            femMod.getSurfaceMesh ().write (chooser.getSelectedFile().getAbsoluteFile(), null);
         }
         catch (FileNotFoundException e1) {
            e1.printStackTrace();
         }
         catch (IOException e1) {
            e1.printStackTrace();
         }
      }
   }

   public boolean getLoadMuscle () {
      return loadMuscle;
   }

   public void setLoadMuscle (boolean loadMuscle) {
      this.loadMuscle = false;
      JFileChooser chooser = new JFileChooser();
      FileNameExtensionFilter filter = new FileNameExtensionFilter(
          "OBJ files", "obj");
      chooser.setFileFilter(filter);
      int returnVal = chooser.showOpenDialog(this.getControlPanelTabs ());
      if(returnVal == JFileChooser.APPROVE_OPTION) {
         try {
            mechMod.axialSprings ().clear ();
            mechMod.frameMarkers ().clear ();
            femMod.clear (); 
            PolygonalMesh msh = new PolygonalMesh(chooser.getSelectedFile().getAbsolutePath ());
            
            FemFactory.createFromMesh (femMod, msh, 0);
            
            for(RigidBody rb : mechMod.rigidBodies ()) {
               attachNearestPoints(femMod, mechMod, rb);
            }
         }
         catch (FileNotFoundException e1) {
            e1.printStackTrace();
         }
         catch (IOException e1) {
            e1.printStackTrace();
         }
      }
   }

   public boolean getReset () {
      return reset;
   }

   public void setReset (boolean reset) {
      this.reset = false;
      mechMod.axialSprings ().clear ();
      mechMod.frameMarkers ().clear ();
      mechMod.remove (femMod);
      femMod = new FemModel3d();
      femMod.setSurfaceRendering (SurfaceRender.Shaded);
      LinearMaterial l = (LinearMaterial)femMod.getMaterial ();
      l.setYoungsModulus (10000);
      mechMod.add (femMod);
      RenderProps.setPointRadius (femMod, this.getMainViewer ().estimateRadiusAndCenter (null) * 0.001);
      RenderProps.setLineStyle (femMod, LineStyle.LINE);
      RenderProps.setLineRadius (femMod, this.getMainViewer ().estimateRadiusAndCenter (null) * 0.01);
      
      mechMod.clearCollisionBehaviors ();
      mechMod.clearCollisionResponses ();
   }

   double distanceInitiale = 0;
   
   double mu = 0.1;
   
   
   public void build (String[] args) throws IOException {
      
      super.build (args);
      
      Main.getMain ().setTimelineVisible (false);
      
      mechMod = new MechModel("surgerymodel");
      addModel(mechMod);
      
      mechMod.setGravity (new Vector3d(0,0,0));
      PolygonalMesh msh;
      
      
      femMod = new FemModel3d();
      femMod.setSurfaceRendering (SurfaceRender.Shaded);
      LinearMaterial l = (LinearMaterial)femMod.getMaterial ();
      l.setYoungsModulus (10000);
      mechMod.add (femMod);
      
      ////
      
      pan = new ControlPanel();
      pan.setName ("Muscle Fitting Control Panel");
      
      pan.addWidget (this, "addBone");
      pan.addWidget (this, "addPoints");
      pan.addWidget (this, "createMuscle");
      pan.addWidget (this, "saveMuscle");
      pan.addWidget (this, "loadMuscle");
      pan.addWidget (this, "reset");
      
      this.addControlPanel (pan);
      
      RenderProps rp = new RenderProps();
      rp.setPointStyle (Renderer.PointStyle.SPHERE);
      rp.setPointColor (Color.LIGHT_GRAY);
      rp.setPointRadius (this.getMainViewer ().estimateRadiusAndCenter (null) * 0.01);
      rp.setLineStyle (Renderer.LineStyle.SPINDLE);
      rp.setLineColor (Color.WHITE);
      rp.setLineRadius (0.4);
      mechMod.setRenderProps (rp);
      this.setRenderProps (rp);
      
      RenderProps.setPointRadius (mechMod, this.getMainViewer ().estimateRadiusAndCenter (null) * 0.01);
      RenderProps.setPointStyle (mechMod, PointStyle.SPHERE);
      RenderProps.setLineStyle (mechMod, LineStyle.LINE);
      
      RenderProps.setPointRadius (femMod, this.getMainViewer ().estimateRadiusAndCenter (null) * 0.001);
      RenderProps.setLineStyle (femMod, LineStyle.LINE);
      RenderProps.setLineRadius (femMod, this.getMainViewer ().estimateRadiusAndCenter (null) * 0.01);
      

      
   }
   
   //fit poutre
   public void createFrame() {
      
      // 1 2
      // 3 4
      // 5 6
      
      //take the 6 framemarkers
      int nframemarkers = this.mechMod.frameMarkers().size();
      if (nframemarkers == 4) {
         createFrame4();
         return;
      }
      else if(nframemarkers < 6) {
         System.out.println("not enough framemarkers"); return;
      }
      
      FrameMarker topleft, topright, botleft, botright, midleft, midright;
      topleft = mechMod.frameMarkers ().get (0);
      topright= mechMod.frameMarkers ().get (1);
      midleft = mechMod.frameMarkers ().get (2);
      midright= mechMod.frameMarkers ().get (3);   
      botleft = mechMod.frameMarkers ().get (4);
      botright= mechMod.frameMarkers ().get (5);
      
      // compute plane with corners
      
      Vector3d diag1 = new Vector3d();
      diag1.sub (topleft.getPosition (), botright.getPosition ());
      
      Vector3d diag2 = new Vector3d();
      diag2.sub (topright.getPosition (), botleft.getPosition ());
      
      Vector3d planeNormal = new Vector3d();
      planeNormal.cross (diag2, diag1);
      
      // compute largeur avec les points du milieu
      
      double largeur = midleft.getPosition ().distance (midright.getPosition ());
      double hauteur = topleft.getPosition ().distance (botleft.getPosition ());
      distanceInitiale = topleft.getPosition ().distance (botright.getPosition ())/2;
      
      //compute  milieu de la poutre
      
      Vector3d middle = new Vector3d();
      middle.add (midleft.getPosition (), midright.getPosition ()).scale (0.5);
      
      middle.add (planeNormal.normalize ().scale (distanceInitiale));
      
      // creer poutre
      
      //femMod.addNode (new FemNode3d(middle.x,middle.y,middle.z));
      
      Vector3d vi = new Vector3d();
      vi.sub (topright.getPosition (), topleft.getPosition ());
      vi.normalize ();
      
      Vector3d vj = new Vector3d();
      vj.sub (topleft.getPosition (), botleft.getPosition ());
      vj.normalize ();
      /*
      int nlarge = 2;
      int nlong = 2;
      int nepaisseur = 3;
      
      for(int k = 0; k < nepaisseur; k++ ) {
         for(int i = -nlarge; i <= nlarge; i ++) {
            for(int j = -nlong; j <= nlong; j++) {
               Point3d point = new Point3d();
               vi.sub (botright.getPosition (), botleft.getPosition ());
               vi.normalize();
               vi.scale (i*largeur/nlarge);
               vj.sub (topleft.getPosition (), botleft.getPosition ());
               vj.normalize();
               vj.scale (j*hauteur/(2*nlong));
               
               point.add (middle,vi);
               point.add (vj);
               if (k > 0) {
                  point.add (planeNormal.normalize ().scale (k*largeur));
               }
               
        //       femMod.addNode (new FemNode3d(point));
            }
         }
      }
      */
      
      //mechMod.addPoint (new Point(new Point3d(middle)));
      
      FemFactory.createGrid (femMod, FemElementType.Tet, largeur*1.2, distanceInitiale*2, largeur/5, 2, 8, 2);
      FemNode3d center = femMod.getNode (13);
      Vector3d v = new Vector3d();
      v.sub (middle, center.getPosition ());
      RotationMatrix3d mat = new RotationMatrix3d();
      mat.setXYDirections (vi, vj);
      femMod.transformPose (new RigidTransform3d(v,mat));
      
      
      //poser poutre...
      femMod.setLinearMaterial (10000, 0, true);
      femMod.setDensity (1);
      
      for (RigidBody rb : mechMod.rigidBodies ()) {
         mechMod.setCollisionBehavior (rb, femMod, true);
         mechMod.setCollisionResponse (rb, femMod);
      }
      

      //createSprings();
      
      AxialSpring spring1, spring2, spring3, spring4, spring5, spring6;
      LinearAxialMaterial l = new LinearAxialMaterial();
      l.setStiffness (1000);
      
      spring1 = new AxialSpring();
      spring1.setFirstPoint (topright);
      spring1.setSecondPoint (femMod.getNode (26));
      spring1.setMaterial (l);
      mechMod.addAxialSpring (spring1);
      
      spring2 = new AxialSpring();
      spring2.setFirstPoint (topleft);
      spring2.setSecondPoint (femMod.getNode (24));
      spring2.setMaterial (l);
      mechMod.addAxialSpring (spring2);
      
      spring3 = new AxialSpring();
      spring3.setFirstPoint (midright);
      spring3.setSecondPoint (femMod.getNode (14));
      spring3.setMaterial (l);
      mechMod.addAxialSpring (spring3);
      
      spring4 = new AxialSpring();
      spring4.setFirstPoint (midleft);
      spring4.setSecondPoint (femMod.getNode (12));
      spring4.setMaterial (l);
      mechMod.addAxialSpring (spring4);
      
      spring5 = new AxialSpring();
      spring5.setFirstPoint (botright);
      spring5.setSecondPoint (femMod.getNode (2));
      spring5.setMaterial (l);
      mechMod.addAxialSpring (spring5);
      
      spring6 = new AxialSpring();
      spring6.setFirstPoint (botleft);
      spring6.setSecondPoint (femMod.getNode (0));
      spring6.setMaterial (l);
      mechMod.addAxialSpring (spring6);
      
   }
   
   void createFrame4() {
      
      FrameMarker topleft, topright, botleft, botright;
      topleft = mechMod.frameMarkers ().get (0);
      topright= mechMod.frameMarkers ().get (1);  
      botleft = mechMod.frameMarkers ().get (2);
      botright= mechMod.frameMarkers ().get (3);
      
      // compute plane with corners
      
      Vector3d diag1 = new Vector3d();
      diag1.sub (topleft.getPosition (), botright.getPosition ());
      
      Vector3d diag2 = new Vector3d();
      diag2.sub (topright.getPosition (), botleft.getPosition ());
      
      Vector3d planeNormal = new Vector3d();
      planeNormal.cross (diag2, diag1);
      
      // compute largeur avec les points du milieu
      
      double largeur = (topleft.getPosition ().distance (topright.getPosition ())+botleft.getPosition ().distance (botright.getPosition ()))/2 ;
      double hauteur = topleft.getPosition ().distance (botleft.getPosition ());
      distanceInitiale = topleft.getPosition ().distance (botright.getPosition ());
      
      //compute  milieu de la poutre
      
      Vector3d middle = new Vector3d();
      middle.add (topleft.getPosition (), botright.getPosition ()).scale (0.5);
      
      middle.add (planeNormal.normalize ().scale (distanceInitiale));
      
      // creer poutre
      
      //femMod.addNode (new FemNode3d(middle.x,middle.y,middle.z));
      
      Vector3d vi = new Vector3d();
      vi.sub (topright.getPosition (), topleft.getPosition ());
      vi.normalize ();
      
      Vector3d vj = new Vector3d();
      vj.sub (topleft.getPosition (), botleft.getPosition ());
      vj.normalize ();
      /*
      int nlarge = 2;
      int nlong = 2;
      int nepaisseur = 3;
      
      for(int k = 0; k < nepaisseur; k++ ) {
         for(int i = -nlarge; i <= nlarge; i ++) {
            for(int j = -nlong; j <= nlong; j++) {
               Point3d point = new Point3d();
               vi.sub (botright.getPosition (), botleft.getPosition ());
               vi.normalize();
               vi.scale (i*largeur/nlarge);
               vj.sub (topleft.getPosition (), botleft.getPosition ());
               vj.normalize();
               vj.scale (j*hauteur/(2*nlong));
               
               point.add (middle,vi);
               point.add (vj);
               if (k > 0) {
                  point.add (planeNormal.normalize ().scale (k*largeur));
               }
               
        //       femMod.addNode (new FemNode3d(point));
            }
         }
      }
      */
      
      //mechMod.addPoint (new Point(new Point3d(middle)));
      
      FemFactory.createGrid (femMod, FemElementType.Tet, largeur*1.2, hauteur, largeur/6, 2, 8, 2);
      FemNode3d center = femMod.getNode (13);
      Vector3d v = new Vector3d();
      v.sub (middle, center.getPosition ());
      RotationMatrix3d mat = new RotationMatrix3d();
      mat.setXYDirections (vi, vj);
      femMod.transformPose (new RigidTransform3d(v,mat));
      
      
      //poser poutre...
      femMod.setLinearMaterial (10000, 0, true);
      femMod.setDensity (1);
      
      for (RigidBody rb : mechMod.rigidBodies ()) {
         mechMod.setCollisionBehavior (rb, femMod, true);
         mechMod.setCollisionResponse (rb, femMod);
      }
      
      //createSprings();
      
      AxialSpring spring1, spring2, spring5, spring6;
      LinearAxialMaterial l = new LinearAxialMaterial();
      l.setStiffness (1000);
      
      spring1 = new AxialSpring();
      spring1.setFirstPoint (topright);
      spring1.setSecondPoint (femMod.getNode (26));
      spring1.setMaterial (l);
      mechMod.addAxialSpring (spring1);
      
      spring2 = new AxialSpring();
      spring2.setFirstPoint (topleft);
      spring2.setSecondPoint (femMod.getNode (24));
      spring2.setMaterial (l);
      mechMod.addAxialSpring (spring2);
      
      spring5 = new AxialSpring();
      spring5.setFirstPoint (botright);
      spring5.setSecondPoint (femMod.getNode (2));
      spring5.setMaterial (l);
      mechMod.addAxialSpring (spring5);
      
      spring6 = new AxialSpring();
      spring6.setFirstPoint (botleft);
      spring6.setSecondPoint (femMod.getNode (0));
      spring6.setMaterial (l);
      mechMod.addAxialSpring (spring6);
      
      
   }
   
   void createSprings() {
      AxialSpring s;
      for(FrameMarker f:mechMod.frameMarkers ()) {
         FemNode3d n = femMod.findNearestNode (f.getPosition (), 10*distanceInitiale);
         s = new AxialSpring();
         s.setFirstPoint (n);
         s.setSecondPoint (f);
         LinearAxialMaterial l = new LinearAxialMaterial();
         l.setStiffness (1000);
         s.setMaterial (l);
         mechMod.addAxialSpring (s);
      }
      
      
   }
   
   
   public static void attachNearestPoints(FemModel3d fem, MechModel mechMod, RigidBody rb) {
      double d = 1;
      for(FemNode3d n : fem.getNodes ()){
         for (Vertex3d v : rb.getMesh ().getVertices ()) {
            double dist = v.getPosition ().distance (new Vector3d(n.getPosition())); 
            if (dist < d) {
               mechMod.attachPoint(n,rb);
            }
         }
      }
      
   }
}
