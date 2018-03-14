package artisynth.core.gui.editorManager;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.event.ActionEvent;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import javax.swing.JLabel;
import javax.swing.JSpinner;
import javax.swing.SpinnerNumberModel;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

import maspack.matrix.Vector3d;

import maspack.matrix.Point2d;

import maspack.geometry.BVFeatureQuery;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;
import maspack.render.MouseRayEvent;
import maspack.render.RenderProps;
import maspack.render.Renderer.LineStyle;
import maspack.render.GL.GLViewer;
import maspack.render.Renderer.Shading;

import artisynth.core.driver.Main;
import artisynth.core.materials.LigamentAxialMaterial;
import artisynth.core.mechmodels.AxialSpring;
import artisynth.core.mechmodels.Collidable.Collidability;
import artisynth.core.mechmodels.Frame;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MultiPointSpring;
import artisynth.core.mechmodels.MultiPointSpringList;
import artisynth.core.mechmodels.Point;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.mechmodels.RigidSphere;
import artisynth.core.mechmodels.SolidJoint;
import artisynth.core.mechmodels.Wrappable;
import artisynth.core.modelbase.ComponentList;
import artisynth.core.modelbase.CompositeComponent;
import artisynth.core.modelbase.ModelComponent;
import artisynth.core.modelbase.MutableCompositeComponent;
import artisynth.core.workspace.RootModel;
import artisynth.models.carapas.Ligament;
import artisynth.models.carapas.Link;
import artisynth.models.carapas.VirtualHipFractureSurgery;

public class CapsuleAgent extends AddComponentAgent implements ChangeListener {

   MechModel myModel;
   VirtualHipFractureSurgery sim;
   
   RigidBody femur;
   
   RigidSphere sphere;
   
   SolidJoint s;
   
   Point3d middle = new Point3d();
   
   State currentState;
   
   List<Link> links;
   
   FrameMarker currentPoint;
   
   private RootModel myLastRootModel;
   private HashMap<Class,ModelComponent> myPrototypeMap;
   
   ComponentList<FrameMarker> firstSet;
   ComponentList<FrameMarker> secondSet;
   
   private JSpinner sphereSizeSpinner;
   
   private enum State {
      FirstPoint, SecondPoint, SelectingFemur, CreatingSphere, Complete
   };
   
   public CapsuleAgent(Main main, VirtualHipFractureSurgery sim) {
      this(main, ((ComponentList<MultiPointSpring>)sim.findComponent("models/surgerymodel/capsulemultipointsprings")), sim.getMechMod ());
      myModel = sim.getMechMod ();
      this.sim = sim;  
      links = new LinkedList<Link>();
      //Ligament proto = (Ligament)this.getPrototypeComponent (Ligament.class);
      //LigamentAxialMaterial mat = (LigamentAxialMaterial)proto.getMaterial ();
      //mat.setElongStiffness (sim.getSacrotuberalStiffness ());
      

   }
   
   public CapsuleAgent (Main main, ComponentList container,
   CompositeComponent ancestor) {
      super (main, container, ancestor);
   }

   @Override
   protected void resetState () {
      goToState(State.FirstPoint);

   }

   @Override
   protected void setInitialState () {
      goToState(State.FirstPoint);
   }

   @Override
   protected void initializePrototype (ModelComponent comp, Class type) {
      // TODO Auto-generated method stub

   }

   @Override
   protected Map getPrototypeMap () {
      RootModel root = myMain.getRootModel();
      if (root != null && root != myLastRootModel) {
         myPrototypeMap = new HashMap<Class,ModelComponent>();
         myLastRootModel = root;
      }
      return myPrototypeMap;
   }
   
   protected void createDisplay() {
      createDisplayFrame ("Create Capsule");
      
      addComponentType(MultiPointSpring.class,new String[] {"name", "restLength", "renderProps", "wrapStiffness", "wrapDamping",
                                                            "contactStiffness", "contactDamping", "maxWrapIterations", "drawKnots",
                                                            "drawABPoints", "sor", "dnrmGain"});
      
      MultiPointSpring mps = (MultiPointSpring) getPrototypeComponent(MultiPointSpring.class);
      mps.setMaterial (new LigamentAxialMaterial());
      ((LigamentAxialMaterial) mps.getMaterial ()).setElongStiffness (sim.getCapsuleStiffness ());
      

      //
      //createSeparator();
      createNameField();
      createTypeSelector ("Spring type");
      //createLigamentSelector();
     
      firstSet = (ComponentList<FrameMarker>)sim.findComponent("models/surgerymodel/capsuleFirstSet");
      secondSet = (ComponentList<FrameMarker>)sim.findComponent ("models/surgerymodel/capsuleSecondSet");

      //createPropertyFrame ("Default TYPE propeties:");
      // createSeparator();
      //createProgressBox();
      createInstructionBox();
      //createContinuousAddToggle();
      JLabel sphereLabel = new JLabel("Sphere Size:");
      SpinnerNumberModel model = new SpinnerNumberModel(0.0,0.0,10000000.0,0.001);
      sphereSizeSpinner = new JSpinner(model);
      sphereSizeSpinner.setValue (0);
      Dimension size = myInstructionBox.getPreferredSize();
      size.width = 1000000;
      sphereSizeSpinner.setMaximumSize (size);
      sphereSizeSpinner.setMinimumSize (size);
      addToContentPane (sphereLabel);
      addToContentPane (sphereSizeSpinner);
      sphereSizeSpinner.addChangeListener (this);
      
      createOptionPanel ("Add Done");
      myAddButton.setText ("Next");
      myDoneButton.setEnabled (false);
      goToState(State.FirstPoint);
      

      installLocationListener();
      
   }
   
   public void handleLocationEvent (GLViewer viewer, MouseRayEvent rayEvent) {
      double mind = Double.POSITIVE_INFINITY;
      RigidBody nearestBody = null;
      Point3d nearestIntersection = null;
      for (RigidBody body : myModel.rigidBodies()) {
         if (body.getMesh() != null) {
            Point3d isectPoint = BVFeatureQuery.nearestPointAlongRay (
               body.getMesh (),
               rayEvent.getRay().getOrigin(), rayEvent.getRay().getDirection());             
            if (isectPoint != null) {
               double d = isectPoint.distance (rayEvent.getRay().getOrigin());
               if (d < mind) {
                  mind = d;
                  nearestBody = body;
                  nearestIntersection = isectPoint;
               }
            }
         }
      }
      if (nearestBody != null) { // transform to body coordinates
         nearestIntersection.inverseTransform (nearestBody.getPose());
         
         switch(currentState) {
            case FirstPoint:
               currentPoint = createAndAddMarker (nearestIntersection, nearestBody, ((ComponentList<FrameMarker>)sim.findComponent("models/surgerymodel/capsuleFirstSet")));
               goToState(State.SecondPoint);
               break;
            case SecondPoint:
               createLink(currentPoint,
                  createAndAddMarker (nearestIntersection, nearestBody, ((ComponentList<FrameMarker>)sim.findComponent("models/surgerymodel/capsuleSecondSet"))));
               goToState(State.FirstPoint);
               break;
            case SelectingFemur:
               femur = nearestBody;
               goToState(State.CreatingSphere);
               break;
            case Complete:
               break;
         }
      }
   }
   
   private void createLink (FrameMarker p1, FrameMarker p2) {
      links.add (new Link(p1,p2));
   }

   private FrameMarker createAndAddMarker (Point3d pnt, Frame frame, MutableCompositeComponent<?> list) {
      FrameMarker marker = new FrameMarker (pnt);
      marker.setFrame (frame);
      marker.setName (getNameFieldValue());
      RenderProps.setPointRadius (marker, getDefaultPointRadius()/2);
      RenderProps.setShading (marker, Shading.SMOOTH);
      RenderProps.setPointColor (marker, Color.RED);
      //setProperties (marker, getPrototypeComponent (myComponentType));
      // update properties in the prototype as well ...
      //setProperties (myPrototype, myPrototype);

      addComponent (new AddComponentsCommand (
         "add FrameMarker", marker,
         list));
      
      return marker;

   }
   
   private void goToState(State s) {
      switch (s) {
         case FirstPoint:
            currentState = State.FirstPoint;
            myAddButton.setEnabled (true);
            sphereSizeSpinner.setEnabled (false);
            myInstructionBox.setText ("Click to add the first point of a link");
            break;
         case SecondPoint:
            currentState = State.SecondPoint;
            myAddButton.setEnabled (false);
            sphereSizeSpinner.setEnabled (false);
            myInstructionBox.setText ("Click to add the second point of a link");
            break;
         case SelectingFemur:
            currentState = State.SelectingFemur;
            myAddButton.setEnabled (false);
            sphereSizeSpinner.setEnabled (false);
            myInstructionBox.setText ("Select the femur");
            break;
         case CreatingSphere:
            currentState = State.CreatingSphere;
            myAddButton.setEnabled (true);
            sphereSizeSpinner.setEnabled (true);
            sphere = createSphere(-1.0);
            sphereSizeSpinner.setValue (sphere.getRadius ());
            myInstructionBox.setText ("Adjust Sphere Size");
            uninstallLocationListener();
            break;
         case Complete:
            currentState = State.Complete;
            myAddButton.setEnabled (false);
            sphereSizeSpinner.setEnabled (false);
            createMultiSprings(sphere);
            myDoneButton.setEnabled (true);
            myInstructionBox.setText ("Click Done");
            RenderProps.setVisible (sphere, false);
            break;
      } 
      
   }


   public void actionPerformed (ActionEvent e) {
      
      if (e.getSource () == myAddButton) {
         switch(currentState) {
            case FirstPoint:
               goToState(State.SelectingFemur);
               break;
            case CreatingSphere:
               goToState(State.Complete);
               break;
         }
         myMain.rerender();
      }
      else if (e.getSource () == myDoneButton) {
         //createAndAddSpring();
         //setState (State.Complete);
         myDisplay.setVisible (false);
         dispose();
         sim.setupRenderProps ();
      }
      else {
         super.actionPerformed (e);
      }
   }
   
   private RigidSphere createSphere(double radius) {
      
      double rayon;
      if (radius > 0) {
         rayon = radius;
      } else {
      Point3d barycentre1 = barycentre(firstSet);
      Point3d barycentre2 = barycentre(secondSet);
      
      Vector3d axe = new Vector3d();
      axe.sub (barycentre1, barycentre2);

      
      Point3d highPoint = getHighestPoint(axe, femur.getMesh (), barycentre2);
      
      //sim.getMechMod ().add (new Point(highPoint));
      
      Point3d[] points = getFarthestPoints(axe,femur.getMesh(), highPoint, barycentre2);
      
         for (int i = 0; i < 4; i++) {
            // System.out.println(i);
            if (points[i] != null)
               sim.getMechMod ().add (new Point (points[i]));
            if (points[i] == null)
               System.out.println ("Could not create capsule like okay idk");
         }

         double distance1 = points[0].distance (points[1]);
         double distance2 = points[2].distance (points[3]);

         Point3d middle1 = new Point3d ();
         Point3d middle2 = new Point3d ();
         int id;
         if (distance1 < distance2)
            id = 0;
         else
            id = 2;

         // middle1.x = (points[0].x + points[1].x )/2;
         // middle1.y = (points[0].y + points[1].y )/2;
         // middle1.z = (points[0].z + points[1].z )/2;
         //
         // middle2.x = (points[2].x + points[3].x )/2;
         // middle2.y = (points[2].y + points[3].y )/2;
         // middle2.z = (points[2].z + points[3].z )/2;
         //
         // middle.x = (middle1.x + middle2.x)/2;
         // middle.y = (middle1.y + middle2.y)/2;
         // middle.z = (middle1.z + middle2.z)/2;

         middle.x = (points[id].x + points[id + 1].x) / 2;
         middle.y = (points[id].y + points[id + 1].y) / 2;
         middle.z = (points[id].z + points[id + 1].z) / 2;

         double[] distances = new double[5];
         for (int i = 0; i < 4; i++)
            distances[i] = middle.distance (points[i]);
         distances[4] = middle.distance (highPoint);

         Arrays.sort (distances);

         rayon = distances[2];
      }
      RigidSphere sphere = new RigidSphere("SphereFemur", rayon,0.1);
      sphere.setPosition (middle);
      sphere.setCollidable (Collidability.OFF);
      sim.getMechMod().add (sphere);
      RenderProps.setVisible (sphere, true);
      
      s = new SolidJoint(femur, sphere);
      s.setName ("CapsuleJoint");
      
      sim.getMechMod().add (s);
      
      
      return sphere;
   }
   
   private void createMultiSprings (RigidSphere sphere) {
      
      MultiPointSpringList<MultiPointSpring> springs = (MultiPointSpringList<MultiPointSpring>)sim.getMechMod ().findComponent ("capsulemultipointsprings");
      
      for(Link l: links) {
         int nsprings = 100;
         MultiPointSpring s = new MultiPointSpring();
         LigamentAxialMaterial m = new LigamentAxialMaterial();
         m.setElongStiffness (sim.getCapsuleStiffness ());
         s.addPoint (l.getFirst ());
         s.setSegmentWrappable (nsprings);
         s.addWrappable (sphere);
         s.addPoint (l.getSecond ());
         s.setMaterial (m);
         s.updateWrapSegments ();
         //s.setRestLength (l.getFirst ().distance (l.getSecond ()));
         s.setRestLengthFromPoints ();
         RenderProps.setLineRadius (s, getDefaultLineRadius()/5);
         RenderProps.setLineStyle (s, LineStyle.CYLINDER);
         RenderProps.setLineColor (s, Color.RED);
         RenderProps.setShading(s,Shading.SMOOTH);
         springs.add (s);
      }
   }
   

   private Point3d barycentre (ComponentList<FrameMarker> set) {
      Point3d bar = new Point3d(0,0,0);
      int n = set.size();
      for(FrameMarker f:set) {
         bar.add (f.getPosition ());
      }
      bar.scale (1.0/(double)n);
      return bar;
   }
   
   private Point3d getHighestPoint(Vector3d axe, PolygonalMesh m, Point3d barycentreBas) {
      Point3d high = new Point3d(barycentreBas);
      double dotp = axe.dot (high);
      
      for(Vertex3d v : m.getVertices ()) {
         double tmpdot = axe.dot (v.getWorldPoint ());
         if(tmpdot > dotp) {
            dotp = tmpdot;
            high = v.getWorldPoint ();
         }
      }
      return high;
   }
   
   private Point3d[] getFarthestPoints(Vector3d axe, PolygonalMesh m, Point3d highPoint, Point3d lowPoint) {
      Point3d[] points = new Point3d[4];
      double[] dotps = new double[4];
      for(int i = 0; i < 4; i++){
         dotps[i] = 0.0;
      }
      
      Vector3d orth = new Vector3d();
      orth.perpendicular (axe);
      
      Vector3d orth2 = new Vector3d();
      orth2.cross (orth, axe);
      
      for(Vertex3d v : m.getVertices ()) {
         Vector3d lpv = new Vector3d();
         lpv.sub (v.getWorldPoint (), lowPoint);
         double angle = axe.angle (lpv);
         if(angle < Math.PI/2 ||  axe.angle (lpv) > 3*Math.PI/2) {
            double tmpdot = orth.dot (lpv);
            if(tmpdot > dotps[0]) {
               dotps[0] = tmpdot;
               points[0] = v.getWorldPoint ();
            }
            if(tmpdot < dotps[1]) {
               dotps[1] = tmpdot;
               points[1] = v.getWorldPoint ();
            }
            tmpdot = orth2.dot (lpv);
            if(tmpdot > dotps[2]) {
               dotps[2] = tmpdot;
               points[2] = v.getWorldPoint ();
            }
            if(tmpdot < dotps[3]) {
               dotps[3] = tmpdot;
               points[3] = v.getWorldPoint ();
            }
         }
      }
      
      return points;
   }

   @Override
   public void stateChanged (ChangeEvent arg0) {
      if(arg0.getSource () == sphereSizeSpinner) {
         if (sphere != null) {
            sim.getMechMod().remove(sphere);
            sim.getMechMod().remove(s);
            sphere = createSphere(((Double)sphereSizeSpinner.getValue()).doubleValue ());
         }
      }
   }
}
