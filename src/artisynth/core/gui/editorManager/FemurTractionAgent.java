package artisynth.core.gui.editorManager;

import java.awt.Color;
import java.awt.event.ActionEvent;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

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

import artisynth.core.driver.Main;
import artisynth.core.materials.LigamentAxialMaterial;
import artisynth.core.mechmodels.AxialSpring;
import artisynth.core.mechmodels.Collidable.Collidability;
import artisynth.core.mechmodels.Frame;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.Particle;
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
import artisynth.models.carapas.Forceps;
import artisynth.models.carapas.Ligament;
import artisynth.models.carapas.Link;
import artisynth.models.carapas.VirtualHipFractureSurgery;

public class FemurTractionAgent extends AddComponentAgent {

   MechModel myModel;
   VirtualHipFractureSurgery sim;
   
   RigidBody femur;
   
   State currentState;
   
   FrameMarker firstPoint, secondPoint, thirdPoint;
   
   private RootModel myLastRootModel;
   private HashMap<Class,ModelComponent> myPrototypeMap;
   
   ComponentList<FrameMarker> pointSet;
   
   private enum State {
      FirstPoint, SecondPoint, ThirdPoint, Complete
   };
   
   public FemurTractionAgent(Main main, VirtualHipFractureSurgery sim) {
      this(main, ((ComponentList<FrameMarker>)sim.findComponent("models/surgerymodel/femurTractionForcepses")), sim.getMechMod ());
      myModel = sim.getMechMod ();
      this.sim = sim;
      //Ligament proto = (Ligament)this.getPrototypeComponent (Ligament.class);
      //LigamentAxialMaterial mat = (LigamentAxialMaterial)proto.getMaterial ();
      //mat.setElongStiffness (sim.getSacrotuberalStiffness ());
      

   }
   
   public FemurTractionAgent (Main main, ComponentList container,
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
      if(type == Forceps.class) {
         comp = new Forceps();
         Forceps f = (Forceps) comp;
         f.setMaterial (null);
      }
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
      createDisplayFrame ("Create Femur Traction");
      
      addComponentType(Forceps.class,new String[] {"name", "restLength", "renderProps"});
      
      Forceps mps = (Forceps) getPrototypeComponent(Forceps.class);
      

      //createSeparator();
      createNameField();
      createTypeSelector ("Spring type");
      //createLigamentSelector();
     
      pointSet = (ComponentList)sim.findComponent("models/surgerymodel/femurTractionPoints");

      createPropertyFrame ("Default TYPE propeties:");
      // createSeparator();
      //createProgressBox();
      createInstructionBox();
      //createContinuousAddToggle();
      createOptionPanel ("Done");
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
         femur = nearestBody;
         switch(currentState) {
            case FirstPoint:
               firstPoint = createAndAddMarker (nearestIntersection, nearestBody, ((ComponentList)sim.findComponent("models/surgerymodel/femurTractionPoints")));
               goToState(State.SecondPoint);
               break;
            case SecondPoint:
               secondPoint = createAndAddMarker (nearestIntersection, nearestBody, ((ComponentList)sim.findComponent("models/surgerymodel/femurTractionPoints")));
               goToState(State.ThirdPoint);
               break;
            case ThirdPoint:
               thirdPoint = createAndAddMarker (nearestIntersection, nearestBody, ((ComponentList)sim.findComponent("models/surgerymodel/femurTractionPoints")));             
               goToState(State.Complete);
               break;
            case Complete:
               break;
         }
      }
   }

   private FrameMarker createAndAddMarker (Point3d pnt, Frame frame, MutableCompositeComponent<?> list) {
      FrameMarker marker = new FrameMarker (pnt);
      marker.setFrame (frame);
      marker.setName (getNameFieldValue());
      RenderProps.setPointRadius (marker, getDefaultPointRadius());
      RenderProps.setPointColor (marker, Color.GRAY);
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
            myInstructionBox.setText ("Click on the Femur Head");
            break;
         case SecondPoint:
            currentState = State.SecondPoint;
            myInstructionBox.setText ("Click on great trochanter");
            break;
         case ThirdPoint:
            currentState = State.ThirdPoint;
            myInstructionBox.setText ("Click on bottom of femur");
            break;
         case Complete:
            createVerticalTraction();
            createSideTraction();
            myDoneButton.setEnabled (true);
            myInstructionBox.setText ("Click Done");
            uninstallLocationListener();
            break;
      } 
      
   }
   
   @SuppressWarnings("unchecked")
   private void createVerticalTraction() {
      
      Vector3d axeVertical = new Vector3d();
      axeVertical.sub (thirdPoint.getPosition (), secondPoint.getPosition ());
      
      Point3d lowPoint = new Point3d();
      lowPoint.add (thirdPoint.getPosition (), axeVertical);
      
      Particle lowPart = new Particle();
      lowPart.setPosition (lowPoint);
      lowPart.setDynamic (false);
      
      ((ComponentList )sim.getMechMod ().get("femurTractionParticles")).add (lowPart);
      RenderProps.setVisible (lowPart, true);
      
      Forceps f = new Forceps();
      f.setFirstPoint (secondPoint);
      f.setSecondPoint (lowPart);
      f.setMaximumTighteningMagnitude (sim.getMaximumTighteningMagnitude ());
      f.setMaterial (null);
      RenderProps.setLineRadius (f, getDefaultLineRadius());
      RenderProps.setLineStyle (f, LineStyle.LINE);
      RenderProps.setLineColor (f, Color.GRAY);
      RenderProps.setPointRadius (lowPart, getDefaultPointRadius());
      
      ((ComponentList )sim.getMechMod ().get("femurTractionForcepses")).add (f);
      
      sim.registerForceps (f);
      
      Main.getMain ().rerender ();
   }
   
   @SuppressWarnings("unchecked")
   private void createSideTraction() {
      
      Point3d middlePoint = new Point3d();
      middlePoint.x = (secondPoint.getPosition ().x + thirdPoint.getPosition ().x )/ 2;
      middlePoint.y = (secondPoint.getPosition ().y + thirdPoint.getPosition ().y )/ 2;
      middlePoint.z = (secondPoint.getPosition ().z + thirdPoint.getPosition ().z )/ 2;
      
      Vector3d axeSide = new Vector3d();
      axeSide.sub (middlePoint, firstPoint.getPosition ());
      
      Particle sidePart = new Particle();
      Point3d partPos = new Point3d();
      partPos.add (middlePoint, axeSide);
      sidePart.setPosition (partPos);
      sidePart.setDynamic (false);
      
      ((ComponentList )sim.getMechMod ().get("femurTractionParticles")).add (sidePart);
      RenderProps.setVisible (sidePart, true);
      
      Forceps f = new Forceps();
      f.setFirstPoint (firstPoint);
      f.setSecondPoint (sidePart);
      f.setMaximumTighteningMagnitude (sim.getMaximumTighteningMagnitude ());
      f.setMaterial (null);
      RenderProps.setLineRadius (f, getDefaultLineRadius());
      RenderProps.setLineStyle (f, LineStyle.LINE);
      RenderProps.setLineColor (f, Color.GRAY);
      RenderProps.setPointRadius (sidePart, getDefaultPointRadius());
      
      ((ComponentList )sim.getMechMod ().get("femurTractionForcepses")).add (f);
      sim.registerForceps (f);
   }


   public void actionPerformed (ActionEvent e) {
      
      if (e.getSource () == myDoneButton) {
         //createAndAddSpring();
         //setState (State.Complete);
         myDisplay.setVisible (false);
         dispose();
      }
      else {
         super.actionPerformed (e);
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
   
}
