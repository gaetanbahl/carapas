/**
 * Copyright (c) 2014, by the Authors: Ian Stavness (UBC)
 *
 * This software is freely available under a 2-clause BSD license. Please see
 * the LICENSE file in the ArtiSynth distribution directory for details.
 */
package artisynth.core.inverse;

import java.awt.Color;
import java.awt.Dimension;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Iterator;

import javax.swing.JLabel;
import javax.swing.JSeparator;

import maspack.interpolation.NumericList;
import maspack.interpolation.NumericListKnot;
import maspack.properties.HasProperties;
import maspack.properties.Property;
import maspack.properties.PropertyInfo;
import maspack.properties.PropertyList;
import maspack.render.RenderProps;
import maspack.render.Renderable;
import artisynth.core.driver.Main;
import artisynth.core.gui.ControlPanel;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MechSystemBase;
import artisynth.core.mechmodels.ExcitationComponent;
import artisynth.core.mechmodels.Frame;
import artisynth.core.mechmodels.MotionTargetComponent;
import artisynth.core.mechmodels.Point;
import artisynth.core.modelbase.Controller;
import artisynth.core.modelbase.Model;
import artisynth.core.modelbase.ModelComponent;
import artisynth.core.probes.NumericInputProbe;
import artisynth.core.probes.NumericOutputProbe;
import artisynth.core.probes.NumericProbeBase;
import artisynth.core.probes.Probe;
import artisynth.core.probes.WayPoint;
import artisynth.core.workspace.RootModel;

public class InverseManager implements HasProperties {

   TrackingController myController;
   public ControlPanel inverseControlPanel = null;

   public static double k = 0;

   public static final double defaultMaxa = 1d;
   public static boolean muscleColoring = true;
   public static double maxa = defaultMaxa;

   public static double defaultProbeDuration = 1.0;
   public static double defaultProbeUpdateInterval = 0.01;
   double myProbeDuration;
   double myProbeUpdateInterval;

   private NumericInputProbe refTargetPosInProbe;
   private NumericInputProbe excitationInput = null;
   private NumericOutputProbe excitationOutProbe;
   private NumericOutputProbe modelTargetPosOutProbe;
   private NumericOutputProbe refTargetPosOutProbe;

   private Main myMain;

   public InverseManager (Main main) {
      myMain = main;
   }

   public static PropertyList myProps = new PropertyList(InverseManager.class);

   static {
      myProps.add("syncTargets", "button to initiate target probe sync", false);
      myProps.add("syncExcitations", "button to initiate target probe sync", false);
      myProps.add(
         "probeDuration", "duration of inverse managed probes",
         defaultProbeDuration);
      myProps.add(
         "probeUpdateInterval", "update interval of inverse managed probes",
         defaultProbeDuration);
   }

   public PropertyList getAllPropertyInfo() {
      return myProps;
   }

   public Property getProperty(String name) {
      return PropertyList.getProperty(name, this);
   }

   public TrackingController getController() {
      return myController;
   }

   public void createController(MechSystemBase m, String name,
      MotionTargetComponent[] modelTargets,
      ExcitationComponent[] exciters) {
      createController(m, name, modelTargets, exciters, 1d, 0.01);
   }

   public void createController(MechSystemBase m, String name,
      MotionTargetComponent[] modelTargets,
      ExcitationComponent[] exciters, double pointRadius, double w2norm) {
      TrackingController invcon = new TrackingController(m, name);
      RenderProps.setPointRadius(invcon, pointRadius);
      for (MotionTargetComponent target : modelTargets)
         invcon.addMotionTarget(target);
      for (ExcitationComponent ex : exciters)
         invcon.addExciter(ex);
      myMain.getRootModel().addController(invcon);

      setController(invcon);
   }

   public void createController(MechSystemBase m) {
      setController(new TrackingController(m));
   }

   public void setController(TrackingController controller) {
      if (controller == null) {
         return;
      }
      myController = controller;
      if (myController.isManaged()) {
         findOrCreateProbes(myMain.getRootModel());
         configureProbes();
      }
      if (myMain.getMainFrame() != null) {
         showInversePanel();
      }
   }

   public void clearContoller() {
      myController.dispose();
      myController = null;
      hideInversePanel();
   }

   public static void setMuscleColoringEnabled(boolean enable) {
      muscleColoring = enable;
      if (!enable) {
         maxa = defaultMaxa; // reset maxa to default
      }
      Main.getMain().rerender();
   }

   public void showInversePanel() {
      if (myController == null) {
         System.err.println("cannot create inverse panel; controller null");
         return;
      }
      hideInversePanel();
      createInverseControlPanel();
      if (!RootModel.isFocusable()) {
         inverseControlPanel.setFocusableWindowState(false);
      }
       inverseControlPanel.setVisible(true);
   }

   public void hideInversePanel() {
      if (inverseControlPanel != null) {
         myMain.deregisterWindow(inverseControlPanel.getFrame());
         inverseControlPanel.dispose();
         inverseControlPanel = null;
      }
   }

   private void removeInverseControlPanel() {
      if (inverseControlPanel != null) {
         myMain.deregisterWindow(inverseControlPanel.getFrame());
         inverseControlPanel.dispose();
         inverseControlPanel = null;
      }
   }

   private void createInverseControlPanel() {
      inverseControlPanel = new ControlPanel("Inverse Control Panel");

      for (PropertyInfo propinfo : getAllPropertyInfo())
         inverseControlPanel.addWidget(this, propinfo.getName());
      for (PropertyInfo propinfo : myController.getAllPropertyInfo())
         inverseControlPanel.addWidget(myController, propinfo.getName());

      for (QPTerm term : myController.getCostTerms()) {
         inverseControlPanel.addWidget(new JSeparator());
         inverseControlPanel.addWidget(new JLabel(term.getClass().getSimpleName()));
         for (PropertyInfo propinfo : term.getAllPropertyInfo ()) {
            inverseControlPanel.addWidget(term,propinfo.getName ());
         }
      }
      
      for (LeastSquaresTerm term : myController.getConstraintTerms()) {
         if (term instanceof LeastSquaresTermBase) {
            inverseControlPanel.addWidget(new JSeparator());
            inverseControlPanel.addWidget(new JLabel(term
               .getClass().getSimpleName()));
            for (PropertyInfo propinfo : ((LeastSquaresTermBase)term)
               .getAllPropertyInfo())
               inverseControlPanel.addWidget(
                  (LeastSquaresTermBase)term, propinfo.getName());
         }
      }

      Dimension d = myMain.getMainFrame().getSize();
      java.awt.Point pos = myMain.getMainFrame().getLocation();
      inverseControlPanel.getFrame().setLocation(pos.x+d.width, pos.y+d.height);

      inverseControlPanel.setScrollable(false);
      myMain.registerWindow(inverseControlPanel.getFrame());
   }

   private void findOrCreateProbes(RootModel root) {
      // root.clearInputProbes();
      // root.clearOutputProbes();

      refTargetPosInProbe = findOrCreateInputProbe(root, "target positions");
      refTargetPosOutProbe = findOrCreateOutputProbe(root, "target positions");
      modelTargetPosOutProbe = findOrCreateOutputProbe(root, "model positions");
      excitationOutProbe =
         findOrCreateOutputProbe(root, "computed excitations");
      excitationInput = findOrCreateInputProbe(root, "input excitations");

      setProbeDuration(defaultProbeDuration);
      setProbeUpdateInterval(defaultProbeUpdateInterval);
   }

   private NumericOutputProbe findOrCreateOutputProbe(RootModel root,
      String name) {
      NumericOutputProbe outProbe;
      Probe p = root.getOutputProbes().get(name);
      if (p != null && p instanceof NumericOutputProbe) {
         outProbe = (NumericOutputProbe)p;
      }
      else {
         outProbe = new NumericOutputProbe();
         outProbe.setName(name);
         root.addOutputProbe(outProbe);
      }
      return outProbe;
   }

   private NumericInputProbe
      findOrCreateInputProbe(RootModel root, String name) {
      NumericInputProbe inProbe;
      Probe p = root.getInputProbes().get(name);
      if (p != null && p instanceof NumericInputProbe) {
         inProbe = (NumericInputProbe)p;
      }
      else {
         inProbe = new NumericInputProbe();
         inProbe.setName(name);
         root.addInputProbe(inProbe);
      }
      return inProbe;
   }

   public void configureProbes() {
      
      if (myController.isManaged()) {
         configureExcitationProbe();
      
         MotionTargetTerm moterm = null;
         for (QPTerm term : myController.getCostTerms()) {
            if (term instanceof MotionTargetTerm) {
               moterm = (MotionTargetTerm)term;
               break;
            }
         }
         for (LeastSquaresTerm term : myController.getConstraintTerms()) {
            if (term instanceof MotionTargetTerm) {
               moterm = (MotionTargetTerm)term;
               break;
            }
         }
         if (moterm != null) {
            configureTargetProbes(
               refTargetPosInProbe, moterm.getTargets(),
               "ref_targetPos_input.txt");
            configureTargetProbes(
               modelTargetPosOutProbe, moterm.getSources(),
               "model_target_position.txt");
            configureTargetProbes(
               refTargetPosOutProbe, moterm.getTargets(),
               "ref_target_position.txt");
         }
      }
   }

   private void configureExcitationProbe() {
      Property[] props = new Property[myController.getExciters().size()];
      for (int i = 0; i < myController.getExciters().size(); i++) {
         // XXX how to handle nested excitations?
         props[i] = myController.getExciters().get(i).getProperty("excitation"); 
      }
      excitationOutProbe.setModel(myController.getMech());
      excitationOutProbe.setOutputProperties(props);
      excitationOutProbe.setAttachedFileName("excitations.txt");

      if (excitationInput != null) {
         excitationInput.setModel(myController.getMech());
         excitationInput.setInputProperties(props);
         excitationInput.setAttachedFileName("excitations_input.txt");
         excitationInput.setActive(false);
      }
   }

   private void configureTargetProbes(NumericProbeBase probe,
      ArrayList<MotionTargetComponent> targets, String filename) {
      ArrayList<Property> props = new ArrayList<Property>();
      for (ModelComponent target : targets) {
         if (target instanceof Point) {
            props.add(target.getProperty("position"));
         }
         else if (target instanceof Frame) {
            props.add(target.getProperty("position"));
            props.add(target.getProperty("orientation"));
         }
         else {
            System.err.println("Unknown target component type: "
               + target.getClass().toString());
         }
      }
      probe.setModel(myController.getMech());
      probe.setAttachedFileName(filename);

      if (probe instanceof NumericInputProbe) {
         ((NumericInputProbe)probe).setInputProperties(props
            .toArray(new Property[props.size()]));
      }
      else if (probe instanceof NumericOutputProbe) {
         ((NumericOutputProbe)probe).setOutputProperties(props
            .toArray(new Property[props.size()]));
      }

      if (probe instanceof NumericInputProbe) {
         File file = probe.getAttachedFile ();
         if (file == null || !file.exists ()) {
            ((NumericInputProbe)probe).loadEmpty ();
            probe.setActive (false);
         }
         else {
            try {
               probe.load ();
            }
            catch (IOException e) {
               e.printStackTrace ();
            }
         }
      }
   }

   public void syncTargetProbes() {
      if (myController.isManaged()) {
         syncProbes(refTargetPosInProbe, modelTargetPosOutProbe);
      }
   }

   public void syncExcitationProbes() {
      if (myController.isManaged() && excitationInput != null) {
         syncProbes(excitationInput, excitationOutProbe);
      }
   }

   private void syncProbes(NumericProbeBase dest, NumericProbeBase source)
   {
      if (dest == null || source == null)
         return;
      syncData(dest.getNumericList(), source.getNumericList());
   }

   private void syncData(NumericList dest, NumericList source)
   {
      if (dest.getVectorSize() != source.getVectorSize())
      {
         System.err.println("syncData - probe data not same length\n source = "
            +
            source.getVectorSize() + ", dest = " + dest.getVectorSize());
         return;
      }
      dest.clear();
      for (Iterator<NumericListKnot> iter = source.iterator(); iter.hasNext();)
      {
         dest.add(new NumericListKnot(iter.next()));
      }
   }

   public double getProbeDuration() {
      return myProbeDuration;
   }

   public void setProbeDuration(double probeDuration) {
      
      if (myController.isManaged()) {
         WayPoint waypoint = myMain.getRootModel().getWayPoint(myProbeDuration);
         if (waypoint != null && waypoint.isBreakPoint()) {
            myMain.getRootModel().removeWayPoint(waypoint);
         }
   
         myProbeDuration = probeDuration;
   
         refTargetPosInProbe.setStopTime (myProbeDuration);
         excitationOutProbe.setStopTime (myProbeDuration);
         modelTargetPosOutProbe.setStopTime (myProbeDuration);
         refTargetPosOutProbe.setStopTime (myProbeDuration);
         if (excitationInput != null) {
            excitationInput.setStopTime (myProbeDuration);
         }
   
         myMain.getRootModel().addBreakPoint(myProbeDuration);
         if (myMain.getTimeline() != null) {
            myMain.getTimeline().requestResetAll();
         }
      }
   }

   public double getProbeUpdateInterval() {
      return myProbeUpdateInterval;
   }

   public void setProbeUpdateInterval(double probeUpdateInterval) {
      myProbeUpdateInterval = probeUpdateInterval;

      refTargetPosInProbe.setUpdateInterval(myProbeUpdateInterval);
      excitationOutProbe.setUpdateInterval(myProbeUpdateInterval);
      modelTargetPosOutProbe.setUpdateInterval(myProbeUpdateInterval);
      refTargetPosOutProbe.setUpdateInterval(myProbeUpdateInterval);
      if (excitationInput != null) {
         excitationInput.setUpdateInterval(myProbeUpdateInterval);
      }
   }

   public boolean getSyncTargets() {
      return false; // only used as "push button"
   }

   public void setSyncTargets(boolean syncTargets) {
      syncTargetProbes();
      // only used as "push button" therefore don't save state
   }
   
   public boolean getSyncExcitations() {
      return false; // only used as "push button"
   }

   public void setSyncExcitations(boolean syncTargets) {
      syncExcitationProbes();
      // only used as "push button" therefore don't save state
   }

   public static boolean inverseControllerExists() {
      return (findInverseController() != null);
   }

   public static TrackingController findInverseController() {
      TrackingController ic = null;
      RootModel root = Main.getMain().getRootModel();
      for (Controller c : root.getControllers()) {
         if (c instanceof TrackingController) {
            ic = (TrackingController)c;
            break;
         }
      }
      return ic;
   }

   public static boolean isInversePanelVisible() {
      Main main = Main.getMain();
      ControlPanel inversePanel = main.getInverseManager().inverseControlPanel;
      return (inversePanel != null);
   }

   public static MechModel findMechModel() {
      Main main = Main.getMain();
      MechModel mech = null;
      for (Model model : main.getRootModel().models()) {
         if (model instanceof MechModel) {
            mech = (MechModel)model;
         }
      }
      return mech;
   }

   public static boolean mechModelExists() {
      return (findMechModel() != null);
   }

   private static float[] hsbTmp = new float[3];

   public static void updateLineColorSaturation(Renderable r, Color m,
      double excitation) {
      Color.RGBtoHSB(m.getRed(), m.getGreen(), m.getBlue(), hsbTmp);
      float saturation =
         (excitation > InverseManager.maxa ? 1f
            : (float)(excitation / InverseManager.maxa));
      RenderProps.setLineColor(
         r, Color.getHSBColor(hsbTmp[0], saturation, hsbTmp[2]));
   }
   
}
