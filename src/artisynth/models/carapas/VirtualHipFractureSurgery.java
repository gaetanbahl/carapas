package artisynth.models.carapas;

import java.awt.Color;
import java.awt.Rectangle;
import java.awt.event.ActionEvent;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.HashMap;

import javax.swing.JButton;
import javax.swing.JFileChooser;
import javax.swing.JPanel;
import javax.swing.JSeparator;
import javax.swing.filechooser.FileNameExtensionFilter;
import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;

import org.w3c.dom.Element;
import org.w3c.dom.NamedNodeMap;
import org.w3c.dom.Node;
import org.xml.sax.SAXException;

import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.Vector3d;
import maspack.properties.PropertyList;
import maspack.render.RenderProps;
import maspack.render.Renderer;
import maspack.render.Renderer.LineStyle;
import maspack.render.Renderer.PointStyle;
import maspack.render.Renderer.Shading;
import maspack.widgets.ButtonCreator;
import maspack.widgets.GuiUtils;
import maspack.widgets.LabeledComponentBase;

import artisynth.core.driver.Main;
import artisynth.core.femmodels.FemFactory;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.gui.ControlPanel;
import artisynth.core.gui.editorManager.AttachmentPointAgent;
import artisynth.core.gui.editorManager.AxialSpringAgent;
import artisynth.core.gui.editorManager.CapsuleAgent;
import artisynth.core.gui.editorManager.FemurTractionAgent;
import artisynth.core.gui.editorManager.ForcepsAgent;
import artisynth.core.gui.editorManager.LigamentAgent;
import artisynth.core.gui.editorManager.MeasureAgent;
import artisynth.core.materials.BlemkerMuscle;
import artisynth.core.materials.LigamentAxialMaterial;
import artisynth.core.materials.LinearMaterial;
import artisynth.core.materials.StVenantKirchoffMaterial;
import artisynth.core.mechmodels.AxialSpring;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MultiPointSpring;
import artisynth.core.mechmodels.MultiPointSpringList;
import artisynth.core.mechmodels.Particle;
import artisynth.core.mechmodels.Point;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.modelbase.ComponentList;
import artisynth.core.workspace.DriverInterface;
import artisynth.core.workspace.RootModel;

public class VirtualHipFractureSurgery extends RootModel {
    
   
   final JFileChooser fc = new JFileChooser();
   
   /////////////////////////////////////////////////////
   //Simulation properties
   /////////////////////////////////////////////////////
   
   double sacrotuberalStiffness = 800000;
   double inguinalStiffness = 100000;
   double capsuleStiffness = 32000000;
   double boneDensity = 1300;
   double attachmentPointsStiffness = 3200000;
   double pullControllerStiffness = 0;
   double maximumTighteningMagnitude = 10000;
   
   boolean addBone = false;
   boolean addLigament = false;
   boolean addForceps = false;
   boolean addAttachmentPoint = false;
   boolean loadXML = false;
   boolean createCapsule = false;
   boolean createFemurTraction = false;
   boolean addFemMuscle = false;
   boolean useMeasureTool = false;
   
   private Color[] colors = {Color.BLUE, Color.RED, Color.CYAN, Color.DARK_GRAY, Color.GREEN, Color.LIGHT_GRAY, Color.MAGENTA, 
                             Color.PINK, Color.ORANGE, Color.WHITE, Color.YELLOW}; 
   
   private int forcepsCounter = 0;
   
   
   static PropertyList simProps = new PropertyList(VirtualHipFractureSurgery.class, RootModel.class);
   
   @Override
   public PropertyList getAllPropertyInfo() {
      return simProps;
   }
   
   static {
      simProps.add ("sacrotuberalStiffness", "Sacrotuberal Stiffness", 800000);
      simProps.add ("inguinalStiffness", "Inguinal Stiffness", 100000);
      simProps.add ("capsuleStiffness", "Capsule Stiffness", 32000000);
      simProps.add ("boneDensity", "Bone Density", 1300);
      simProps.add ("attachmentPointsStiffness", "Attachment Points Stiffness", 3200000);
      simProps.add ("pullControllerStiffness", "Pull Controller Stiffness", 30000000);
      simProps.add ("maximumTighteningMagnitude", "Maximum Forceps Tightening Magnitude", 10000);
      simProps.add ("AddBone","Add Bone", false);
      simProps.add ("AddLigament","Add Ligament", false);
      simProps.add ("AddForceps","Add Forceps", false);
      simProps.add ("AddAttachmentPoint","Add Attachment Point", false);
      simProps.add ("LoadXML","Load XML", false);
      simProps.add ("createCapsule", "Create Capsule", false);
      simProps.add ("createFemurTraction", "Create Femur Traction", false);
      simProps.add ("addFemMuscle", "Add Fem Muscle", false);
      simProps.add ("useMeasureTool", "Use Measure Tool", false);
   }

   public double getSacrotuberalStiffness () {
      return sacrotuberalStiffness;
   }

   public void setSacrotuberalStiffness (double sacrotuberalStiffness) {
      this.sacrotuberalStiffness = sacrotuberalStiffness;
      updateLigamentsStiffness();
   }

   public double getInguinalStiffness () {
      return inguinalStiffness;
   }

   public void setInguinalStiffness (double inguinalStiffness) {
      this.inguinalStiffness = inguinalStiffness;
      updateLigamentsStiffness();
   }

   public double getCapsuleStiffness () {
      return capsuleStiffness;
   }

   public void setCapsuleStiffness (double capsuleStiffness) {
      this.capsuleStiffness = capsuleStiffness;
      updateLigamentsStiffness();
   }

   public double getBoneDensity () {
      return boneDensity;
   }

   public void setBoneDensity (double boneDensity) {
      for (RigidBody b : getMechMod().rigidBodies ()) {
         b.setDensity (boneDensity);
      }
      this.boneDensity = boneDensity;
   }
   

   public double getAttachmentPointsStiffness () {
      return attachmentPointsStiffness;
   }

   public void setAttachmentPointsStiffness (double attachmentPointsStiffness) {
      this.attachmentPointsStiffness = attachmentPointsStiffness;
      updateAttachmentPointStiffness();
   }
   

   public double getMaximumTighteningMagnitude () {
      return maximumTighteningMagnitude;
   }

   public void setMaximumTighteningMagnitude (double maximumTighteningMagnitude) {
      this.maximumTighteningMagnitude = maximumTighteningMagnitude;
      updateForcepsMaximumMagnitude();
   }


   public double getPullControllerStiffness () {
      return pullControllerStiffness;
   }

   public void setPullControllerStiffness (double pullControllerStiffness) {
      Main.getMain ().setPullControllerStiffness (pullControllerStiffness);
      this.pullControllerStiffness = pullControllerStiffness;
   }
   
   /////////////////////////////////////////////////////
   /////////////////////////////////////////////////////

   public boolean getAddBone () {
      return addBone;
   }

   public void setAddBone (boolean addBone) {
      this.addBone = false;
      BoneAgent agent = new BoneAgent(Main.getMain (),this);
      agent.show (new Rectangle(640,480,640,480));
   }

   public boolean getAddLigament () {
      return addLigament;
   }

   public void setAddLigament (boolean addLigament) {
      this.addLigament = false;
      LigamentAgent<AxialSpring> agent = new LigamentAgent<AxialSpring>(Main.getMain (),this);
      agent.show (new Rectangle(640,480,640,480));
   }

   public boolean getAddForceps () {
      return addForceps;
   }

   public void setAddForceps (boolean addForceps) {
      this.addForceps = false;
      ForcepsAgent<AxialSpring> agent = new ForcepsAgent<AxialSpring>(Main.getMain (),this);
      agent.show (new Rectangle(640,480,640,480));
   }

   public boolean getAddAttachmentPoint () {
      return addAttachmentPoint;
   }

   public void setAddAttachmentPoint (boolean addAttachmentPoint) {
      this.addAttachmentPoint = false;
      AttachmentPointAgent<AxialSpring> agent = new AttachmentPointAgent<AxialSpring>(Main.getMain (),this);
      agent.show (new Rectangle(640,480,640,480));
   }

   public boolean getLoadXML () {
      return loadXML;
   }

   public void setLoadXML (boolean loadXml) {
      this.loadXML = false;
      int returnVal = fc.showOpenDialog(new JPanel());
      
      if (returnVal == JFileChooser.APPROVE_OPTION) {
          File file = fc.getSelectedFile();
          loadConstraintsFromXml(file.getAbsolutePath ());
      } else {
          System.out.println("Open command cancelled by user.");
      }
   }

   public boolean getCreateCapsule () {
      return createCapsule;
   }

   public void setCreateCapsule (boolean createCapsule) {
      //this.createCapsule = false;
      if(getMechMod() != null) {
         if(createCapsule) {
            
            ComponentList<FrameMarker> firstSet = new ComponentList<FrameMarker>(FrameMarker.class);
            ComponentList<FrameMarker> secondSet = new ComponentList<FrameMarker>(FrameMarker.class);
            
            if(getMechMod().get ("capsuleFirstSet") == null) {
               firstSet.setName ("capsuleFirstSet");
               getMechMod().add (firstSet);
            }
            
            if(getMechMod().get ("capsuleSecondSet") == null) {
               secondSet.setName ("capsuleSecondSet");
               getMechMod().add (secondSet);
            }
            
            CapsuleAgent agent = new CapsuleAgent(Main.getMain (),this);
            agent.show (new Rectangle(640,480,640,480));
         } else {
            getMechMod().remove (getMechMod().get("SphereFemur"));
            getMechMod().remove (getMechMod().get("CapsuleJoint"));
            getMechMod().remove (getMechMod().get("capsuleFirstSet"));
            getMechMod().remove (getMechMod().get("capsuleSecondSet"));
            MultiPointSpringList m = (MultiPointSpringList)getMechMod().get("capsulemultipointsprings");
            m.removeAll ();
         }
      }
      
      this.createCapsule = createCapsule;
   }

   public boolean getCreateFemurTraction () {
      return createFemurTraction;
   }

   @SuppressWarnings("unchecked")
   public void setCreateFemurTraction (boolean createFemurTraction) {
      this.createFemurTraction = createFemurTraction;
      if(getMechMod() != null) {
         if(createFemurTraction) {
            ComponentList<FrameMarker> femurTractionPoints = new ComponentList<FrameMarker>(FrameMarker.class);
            femurTractionPoints.setName ("femurTractionPoints");
            ComponentList<Forceps> femurTractionForcepses = new ComponentList<Forceps>(Forceps.class);
            femurTractionForcepses.setName ("femurTractionForcepses");
            ComponentList<Particle> femurTractionParticles = new ComponentList<Particle>(Particle.class);
            femurTractionParticles.setName ("femurTractionParticles");
            
            getMechMod().add (femurTractionPoints);
            getMechMod().add (femurTractionForcepses);
            getMechMod().add (femurTractionParticles);
            
            FemurTractionAgent agent = new FemurTractionAgent(Main.getMain (),this);
            agent.show (new Rectangle(640,480,640,480));
         } else {
            getMechMod().remove (getMechMod().get("femurTractionPoints"));
            getMechMod().remove (getMechMod().get("femurTractionForcepses"));
            getMechMod().remove (getMechMod().get("femurTractionParticles"));
         }
      }
   }

   public boolean getAddFemMuscle () {
      return addFemMuscle;
   }

   public void setAddFemMuscle (boolean addFemMuscle) {
      
      FemModel3d femMod;
      
      JFileChooser chooser = new JFileChooser();
      FileNameExtensionFilter filter = new FileNameExtensionFilter(
          "OBJ files", "obj");
      chooser.setFileFilter(filter);
      int returnVal = chooser.showOpenDialog(this.getMainFrame ());
      if(returnVal == JFileChooser.APPROVE_OPTION) {
         try {
            femMod = new FemModel3d();
            
            PolygonalMesh msh = new PolygonalMesh(chooser.getSelectedFile().getAbsolutePath ());
            
            FemFactory.createFromMesh (femMod, msh, 0);
            
            getMechMod().add (femMod);
            
            for(RigidBody rb : getMechMod().rigidBodies ()) {
               attachNearestPoints(femMod, getMechMod(), rb);
            }
            
            femMod.setDensity (1100);
            femMod.setMaterial (new LinearMaterial(3000000,0.49));
            femMod.setSurfaceRendering (SurfaceRender.Shaded);
            
            RenderProps.setShading (femMod, Shading.SMOOTH);
            RenderProps.setEdgeColor (femMod, Color.WHITE);
            RenderProps.setPointStyle(femMod, PointStyle.POINT);
            RenderProps.setFaceColor (femMod, Color.decode ("#EC0000"));
            RenderProps.setShininess (femMod, 50);
            
            getMainViewer().rerender ();
         }
         catch (FileNotFoundException e1) {
            // TODO Auto-generated catch block
            e1.printStackTrace();
         }
         catch (IOException e1) {
            // TODO Auto-generated catch block
            e1.printStackTrace();
         }
      }
      this.addFemMuscle = false;
   }
   
   void attachNearestPoints(FemModel3d fem, MechModel mechMod, RigidBody rb) {
      double d = 0.001;
      for(FemNode3d n : fem.getNodes ()){
         for(Vertex3d v : rb.getMesh ().getVertices ())  {
            double dist = v.getPosition ().distance (new Vector3d(n.getPosition())); 
            if (dist < d) {
               mechMod.attachPoint(n,rb);
               break;
            }
         }
      }
   }

   public boolean getUseMeasureTool () {
      return useMeasureTool;
   }

   public void setUseMeasureTool (boolean useMeasureTool) {
      this.useMeasureTool = false;
      
      MeasureAgent agent = new MeasureAgent(Main.getMain (), this);
      agent.show (new Rectangle(640,480,640,480));
   }

   MechModel mechMod; 
   ControlPanel simSettingsPanel;

   /////////////////////////////////////////////////////
   /////////////////////////////////////////////////////
   
   RenderProps ligamentRenderProps;
   RenderProps forcepsRenderProps;
   RenderProps boneRenderProps;
   RenderProps attachmentRenderProps;
   double pointRadius = 0.015;
   double lineRadius = 0.01;
   
   
   public VirtualHipFractureSurgery() {
      super();
   }
   
   public void build(String[] args) throws IOException {
      
      super.build (args);

      Main.getMain ().setTimelineVisible (false);
      
      mechMod = new MechModel("surgerymodel");
      addModel(mechMod);
      
      //set basic settings
      mechMod.setGravity (new Vector3d(0,0,0));
      mechMod.setRotaryDamping (100000);
      mechMod.setFrameDamping (5000000);
      mechMod.setMaxStepSize (0.005);
      
      RenderProps rp = new RenderProps();
      rp.setPointStyle (Renderer.PointStyle.SPHERE);
      rp.setPointColor (Color.LIGHT_GRAY);
      rp.setPointRadius (this.getMainViewer ().estimateRadiusAndCenter (null) * 0.01);
      rp.setLineStyle (Renderer.LineStyle.SPINDLE);
      rp.setLineColor (Color.WHITE);
      rp.setLineRadius (0.4);
      mechMod.setRenderProps (rp);
      this.setRenderProps (rp);
      
      ComponentList<AxialSpring> sacrotuberalLigaments = new ComponentList<AxialSpring>(AxialSpring.class);
      ComponentList<AxialSpring> inguinalLigaments = new ComponentList<AxialSpring>(AxialSpring.class);
      ComponentList<AxialSpring> capsuleLigaments = new ComponentList<AxialSpring>(AxialSpring.class);
      ComponentList<AxialSpring> attachmentPoints = new ComponentList<AxialSpring>(AxialSpring.class);
      ComponentList<AxialSpring> otherLigaments = new ComponentList<AxialSpring>(AxialSpring.class);
      
      sacrotuberalLigaments.setName ("sacrotuberalLigaments");
      inguinalLigaments.setName ("inguinalLigaments");
      capsuleLigaments.setName ("capsuleLigaments");
      attachmentPoints.setName ("attachmentPoints");
      otherLigaments.setName ("otherLigaments");
      
      mechMod.add (sacrotuberalLigaments);
      mechMod.add (inguinalLigaments);
      mechMod.add (capsuleLigaments);
      mechMod.add (attachmentPoints);
      mechMod.add (otherLigaments);
      
      //add panel for simulation settings
      
      simSettingsPanel = new ControlPanel("Simulation Settings");
      simSettingsPanel.setName ("simulationSettings");
      
      simSettingsPanel.addLabel("Add Components");
      
      simSettingsPanel.addWidget (this, "AddBone");
      simSettingsPanel.addWidget (this, "AddLigament");
      simSettingsPanel.addWidget (this, "AddForceps");
      simSettingsPanel.addWidget (this, "AddAttachmentPoint");
      simSettingsPanel.addWidget (this, "LoadXML");
      simSettingsPanel.addWidget (this, "createCapsule");
      simSettingsPanel.addWidget (this, "createFemurTraction");
      simSettingsPanel.addWidget (this, "addFemMuscle");
      simSettingsPanel.addWidget (this, "useMeasureTool");
      
      simSettingsPanel.addLabel ("General Simulation Settings");
      simSettingsPanel.addWidget (mechMod, "frameDamping");
      simSettingsPanel.addWidget (mechMod, "rotaryDamping");
      simSettingsPanel.addWidget (mechMod, "maxStepSize");
      
      simSettingsPanel.addWidget (new JSeparator());
      
      simSettingsPanel.addLabel ("Global Bone Settings");
      simSettingsPanel.addWidget (this, "boneDensity");
      
      simSettingsPanel.addWidget (new JSeparator());
      
      simSettingsPanel.addLabel ("Global Forceps Settings");
      simSettingsPanel.addWidget (this, "maximumTighteningMagnitude");
      
      simSettingsPanel.addWidget (new JSeparator());
      
      simSettingsPanel.addLabel ("Global Ligament Settings");
      simSettingsPanel.addWidget (this, "sacrotuberalStiffness");
      simSettingsPanel.addWidget (this, "inguinalStiffness");
      simSettingsPanel.addWidget (this, "capsuleStiffness");
      
      simSettingsPanel.addWidget (new JSeparator());
      
      simSettingsPanel.addLabel ("Global Attachment Points Settings");
      simSettingsPanel.addWidget (this, "attachmentPointsStiffness");
      
      simSettingsPanel.addWidget (new JSeparator());
      
      simSettingsPanel.addLabel ("Pull Interactor");
      simSettingsPanel.addWidget (this, "pullControllerStiffness");
      
      
      simSettingsPanel.addWidget (new JSeparator());
      
      simSettingsPanel.addLabel ("Forcepses");
      this.addControlPanel (simSettingsPanel);
      
      
      mechMod.setPenetrationTol (0.0001);
      mechMod.setDefaultCollisionBehavior (true, 0);
      
      Main.getMain ().setPullControllerStiffness (this.getPullControllerStiffness ());
      Main.getMain ().getPullController ().getRenderProps ().setPointRadius (this.getMainViewer ().estimateRadiusAndCenter (null) * 0.01);
      //Main.getMain ().getPullController ().getRenderProps ().setPointStyle (PointStyle.POINT);
      
      
      MultiPointSpringList multipointsprings = new MultiPointSpringList(MultiPointSpring.class);
      multipointsprings.setName ("capsulemultipointsprings");
      mechMod.add (multipointsprings);
     
   }
   
   public void setupRenderProps() {
      RenderProps rp = new RenderProps();
      rp.setPointStyle (Renderer.PointStyle.SPHERE);
      rp.setPointColor (Color.LIGHT_GRAY);
      rp.setPointRadius (this.getMainViewer ().estimateRadiusAndCenter (null) * 0.01);
      rp.setLineStyle (Renderer.LineStyle.SPINDLE);
      rp.setLineColor (Color.WHITE);
      rp.setLineRadius (0.4);
      mechMod.setRenderProps (rp);
      this.setRenderProps (rp);
   }
   
   public MechModel getMechMod () {
      if(mechMod == null) {
         mechMod = (MechModel)this.models ().get ("surgerymodel");
      }
      
      return mechMod;
   }

   public void setMechMod (MechModel mechMod) {
      this.mechMod = mechMod;
   }

   public RenderProps getLigamentRenderProps () {
      return ligamentRenderProps;
   }

   public void setLigamentRenderProps (RenderProps ligamentRenderProps) {
      this.ligamentRenderProps = ligamentRenderProps;
   }

   public RenderProps getForcepsRenderProps () {
      return forcepsRenderProps;
   }

   public void setForcepsRenderProps (RenderProps forcepsRenderProps) {
      this.forcepsRenderProps = forcepsRenderProps;
   }

   public RenderProps getBoneRenderProps () {
      return boneRenderProps;
   }

   public void setBoneRenderProps (RenderProps boneRenderProps) {
      this.boneRenderProps = boneRenderProps;
   }

   public RenderProps getAttachmentRenderProps () {
      return attachmentRenderProps;
   }

   public void setAttachmentRenderProps (RenderProps attachmentRenderProps) {
      this.attachmentRenderProps = attachmentRenderProps;
   }

   public void actionPerformed(ActionEvent e)
   {    
      if(e.getActionCommand () == "Add Bone") {
         BoneAgent agent = new BoneAgent(Main.getMain (),this);
         agent.show (new Rectangle(640,480,640,480));
      } else if(e.getActionCommand () == "Add Ligament") {
         LigamentAgent<AxialSpring> agent = new LigamentAgent<AxialSpring>(Main.getMain (),this);
         agent.show (new Rectangle(640,480,640,480));
      } else if(e.getActionCommand () == "Add Forceps") {
         ForcepsAgent<AxialSpring> agent = new ForcepsAgent<AxialSpring>(Main.getMain (),this);
         agent.show (new Rectangle(640,480,640,480));
      } else if(e.getActionCommand () == "Add Attachment Point") {
         AttachmentPointAgent<AxialSpring> agent = new AttachmentPointAgent<AxialSpring>(Main.getMain (),this);
         agent.show (new Rectangle(640,480,640,480));
      } else if (e.getActionCommand () == "Load XML") {
         int returnVal = fc.showOpenDialog(new JPanel());
         
         if (returnVal == JFileChooser.APPROVE_OPTION) {
             File file = fc.getSelectedFile();
             loadConstraintsFromXml(file.getAbsolutePath ());
         } else {
             System.out.println("Open command cancelled by user.");
         }
     }
   }
   
   @SuppressWarnings("unchecked")
   public void updateLigamentsStiffness() {
      if(getMechMod() != null) {
         for(AxialSpring a: (ComponentList<AxialSpring>)findComponent ("models/surgerymodel/sacrotuberalLigaments")) {
            LigamentAxialMaterial l = (LigamentAxialMaterial)a.getMaterial ();
            l.setElongStiffness (getSacrotuberalStiffness());
         }
         for(AxialSpring a: (ComponentList<AxialSpring>)findComponent("models/surgerymodel/inguinalLigaments")) {
            LigamentAxialMaterial l = (LigamentAxialMaterial)a.getMaterial ();
            l.setElongStiffness (getInguinalStiffness());
         }
         for(AxialSpring a: (ComponentList<AxialSpring>)findComponent("models/surgerymodel/capsuleLigaments")) {
            LigamentAxialMaterial l = (LigamentAxialMaterial)a.getMaterial ();
            l.setElongStiffness (getCapsuleStiffness()/((ComponentList<AxialSpring>)findComponent("models/surgerymodel/capsuleLigaments")).size ());
         }
         for(AxialSpring a: (ComponentList<AxialSpring>)findComponent("models/surgerymodel/attachmentPoints")) {
            LigamentAxialMaterial l = (LigamentAxialMaterial)a.getMaterial ();
            l.setElongStiffness (getAttachmentPointsStiffness());
         }
         
         ComponentList<MultiPointSpring> capsulemultipointsprings = ((ComponentList<MultiPointSpring>)findComponent("models/surgerymodel/capsulemultipointsprings"));
         if(capsulemultipointsprings != null) {
            for (MultiPointSpring f : capsulemultipointsprings) {
               LigamentAxialMaterial l = (LigamentAxialMaterial)f.getMaterial ();
               l.setElongStiffness (capsuleStiffness);
            }
         }
      }
   }
   
   public void updateAttachmentPointStiffness() {
      if(getMechMod() != null) {
         ComponentList<AxialSpring> cl = (ComponentList<AxialSpring>)findComponent("models/surgerymodel/attachmentPoints");
         if(cl == null)
            return;
         
         for(AxialSpring a: cl) {
            LigamentAxialMaterial l = (LigamentAxialMaterial)a.getMaterial ();
            l.setElongStiffness (getAttachmentPointsStiffness());
         }
      }
   }
   

   private void updateForcepsMaximumMagnitude () {
      if(getMechMod() != null) {
         for (AxialSpring a : getMechMod().axialSprings ()) {
            if(a instanceof Forceps) {
               Forceps f = (Forceps)a;
               f.setMaximumTighteningMagnitude (maximumTighteningMagnitude);
            }
         }
         ComponentList<Forceps> femurTractionForcepses = ((ComponentList<Forceps>)findComponent("models/surgerymodel/femurTractionForcepses"));
         if(femurTractionForcepses != null) {
            for (Forceps f : femurTractionForcepses) {
               f.setMaximumTighteningMagnitude (maximumTighteningMagnitude);
            }
         }
      }
   }
   
   @SuppressWarnings("unchecked")
   public void registerLigament(AxialSpring l, String typ) {
      if(typ.equals ("Sacrotuberal")){
         ((ComponentList<AxialSpring>)findComponent("models/surgerymodel/sacrotuberalLigaments")).add (l);
      }
      else if(typ.equals ("Inguinal")){
         ((ComponentList<AxialSpring>)findComponent("models/surgerymodel/inguinalLigaments")).add (l);
      }
      else if(typ.equals ("Capsule")){
         ((ComponentList<AxialSpring>)findComponent("models/surgerymodel/capsuleLigaments")).add (l);
      }
      else if(typ.equals ("Attachment Point")){
         ((ComponentList<AxialSpring>)findComponent("models/surgerymodel/attachmentPoints")).add (l);
      }
      else {
         ((ComponentList<AxialSpring>)findComponent("models/surgerymodel/otherLigaments")).add (l);
      }
      updateLigamentsStiffness();
   }

   public void registerForceps(Forceps f) {
      if(simSettingsPanel == null) {
         simSettingsPanel = this.getControlPanels ().get ("simulationSettings");
      }
      LabeledComponentBase widget = simSettingsPanel.addWidget (f, "tightening", 0, 100);
      simSettingsPanel.addWidget (new JSeparator());
      simSettingsPanel.pack();
      
      
      widget.setLabelFontColor (colors[forcepsCounter % colors.length]);
      RenderProps.setLineColor (f, colors[forcepsCounter % colors.length]);
      
      forcepsCounter++;
   }
   
   public void scaleMesh (RigidBody rigidBody, double scalingFactor)
   {
      PolygonalMesh mesh = rigidBody.getMesh ();
      mesh.scale (scalingFactor);
      rigidBody.setMesh (mesh);
   }
   
   private void loadConstraintsFromXml(String filename) {

      final DocumentBuilderFactory factory = DocumentBuilderFactory.newInstance();
              
      try {
          final DocumentBuilder builder = factory.newDocumentBuilder();
                
          final org.w3c.dom.Document document= builder.parse(new File(filename));
          
          final Element racine = document.getDocumentElement();
         
          Node n = racine.getFirstChild ();
          
          while(n != null) {
             if(n.getNodeName () == "ligament" || n.getNodeName () == "forceps") {
                NamedNodeMap attr = n.getAttributes ();
                String mesh1 = attr.getNamedItem ("mesh1").getNodeValue();
                String mesh2 = attr.getNamedItem ("mesh2").getNodeValue();
                
                int index1 = Integer.parseInt (attr.getNamedItem("index1").getNodeValue());
                int index2 = Integer.parseInt (attr.getNamedItem("index2").getNodeValue());
                boolean enabled = Integer.parseInt (attr.getNamedItem("enabled").getNodeValue()) == 1;
                
                
                RigidBody rb1 = null, rb2 = null;
                for(RigidBody rb: getMechMod().rigidBodies()) {
                   if(rb.getName().equals (mesh1)) {
                      rb1 = rb;
                   }
                   else if(rb.getName().equals(mesh2)) {
                      rb2 = rb;
                   }
                }
                if(rb1 == null || rb2 == null) {
                   System.out.println("Erreur XML");
                }
                if(n.getNodeName () == "ligament")
                {
                   String type = attr.getNamedItem ("type").getNodeValue(); 
                   Ligament l = legacyAddLigament("", rb1, index1, rb2, index2, type);
                   //l.setEnabled(enabled);
                }
                else if(n.getNodeName () == "forceps")
                {
                   Forceps f = legacyAddForceps("", rb1, index1, rb2, index2);
                   //f.setEnabled (enabled);
                }
             }
             else if(n.getNodeName () == "attachment_point") {
                NamedNodeMap attr = n.getAttributes ();
                String mesh = attr.getNamedItem ("mesh").getNodeValue();
                
                int index = Integer.parseInt (attr.getNamedItem("index").getNodeValue());
                boolean enabled = Integer.parseInt (attr.getNamedItem("enabled").getNodeValue()) == 1;
                
                RigidBody rb1 = null;
                for(RigidBody rb: getMechMod().rigidBodies()) {
                   if(rb.getName().equals(mesh)) {
                      rb1 = rb;
                   }
                }
                
                Ligament l = legacyAddAttachmentPoint("", rb1, index);
             }
             else if(n.getNodeName () == "rigid_body") {
                NamedNodeMap attr = n.getAttributes ();
                String mesh = attr.getNamedItem ("mesh").getNodeValue();
                
                boolean dynamic = Integer.parseInt (attr.getNamedItem("dynamic").getNodeValue()) == 1;
                for(RigidBody rb: getMechMod().rigidBodies()) {
                   if(rb.getName().equals(mesh)) {
                      rb.setDynamic (dynamic);
                   }
                }
             }
             
             n = n.getNextSibling ();
          }
                    
      }
      catch (final ParserConfigurationException e) {
          e.printStackTrace();
      }
      catch (final SAXException e) {
          e.printStackTrace();
      }
      catch (final IOException e) {
          e.printStackTrace();
      }       
   
   }
   
   public void setCorrectPointRadius(Point p) {
      p.setRenderProps (p.createRenderProps ());
      p.getRenderProps ().setPointRadius (this.getMainViewer ().estimateRadiusAndCenter (null) * pointRadius);
   }
   
   public void setCorrectLineRadius(AxialSpring p) {
      p.setRenderProps (p.createRenderProps ());
      p.getRenderProps ().setLineRadius (this.getMainViewer ().estimateRadiusAndCenter (null) * lineRadius);
   }
   
   public Ligament legacyAddLigament (String name, RigidBody rigidBody1, int index1, RigidBody rigidBody2, int index2, String type) {
      if (rigidBody1 != null && rigidBody2 != null)
      {
         try 
         {
            Ligament ligament = new Ligament ();
            ligament.setName (name);

            Vertex3d v1 = rigidBody1.getMesh ().getVertex (index1);
            Vertex3d v2 = rigidBody2.getMesh ().getVertex (index2);

            if (v1 == null || v2 == null) 
               return null;

            FrameMarker p1 = new FrameMarker (v1.getPosition ());
            p1.setFrame (rigidBody1);
            ligament.setFirstPoint (p1);

            FrameMarker p2 = new FrameMarker (v2.getPosition ());
            p2.setFrame (rigidBody2);
            ligament.setSecondPoint (p2);
            
            this.setCorrectPointRadius(p1);
            this.setCorrectPointRadius (p2);
            this.setCorrectLineRadius(ligament);
            
            RenderProps.setShading (p1, Shading.SMOOTH);
            RenderProps.setPointColor (p1, Color.decode ("#FFCCCC"));
            

            RenderProps.setShading (p2, Shading.SMOOTH);
            RenderProps.setPointColor (p2, Color.decode ("#FFCCCC"));
            

            getMechMod().addFrameMarker (p1);
            getMechMod().addFrameMarker (p2);
            
            LigamentAxialMaterial mat = new LigamentAxialMaterial();
            ligament.setRestLengthFromPoints ();
            ligament.setMaterial (mat);
            if(type.equals ("Sacrotuberal".toUpperCase ())) {
               mat.setElongStiffness (getSacrotuberalStiffness ());
               this.registerLigament (ligament, "Sacrotuberal");
            }
            else if(type.equals ("Inguinal".toUpperCase ())) {
               mat.setElongStiffness (getInguinalStiffness ());
               this.registerLigament (ligament, "Inguinal");
            } 
            else if(type.equals ("Capsule".toUpperCase ())) {
               mat.setElongStiffness (getCapsuleStiffness ());
               this.registerLigament (ligament, "Capsule");
            }
            
            RenderProps.setLineColor (ligament, Color.decode("#FFCCCC"));
            RenderProps.setShading (ligament, Shading.SMOOTH);

            
            
            //getMechMod().addAxialSpring (ligament);
            
            return ligament;
         } catch (Exception e) {
            e.printStackTrace();
         }
      }

      return null;
   }
   
   public Forceps legacyAddForceps (
      String name, RigidBody rigidBody1, int index1, RigidBody rigidBoby2, int index2)
   {
      if (rigidBody1 != null && rigidBoby2 != null)
      {
         try {
            Forceps forceps = new Forceps ();
            forceps.setName (name);

            Vertex3d v1 = rigidBody1.getMesh ().getVertex (index1);
            Vertex3d v2 = rigidBoby2.getMesh ().getVertex (index2);

            if (v1 == null || v2 == null)
               return null;
            
            forceps.setMaterial (null);

            FrameMarker p1 = new FrameMarker (v1.getPosition ());
            p1.setFrame (rigidBody1);
            forceps.setFirstPoint (p1);

            FrameMarker p2 = new FrameMarker (v2.getPosition ());
            p2.setFrame (rigidBoby2);
            forceps.setSecondPoint (p2);

            this.setCorrectPointRadius(p1);
            this.setCorrectPointRadius (p2);
            this.setCorrectLineRadius(forceps);

            //forceps.updateAxis ();
            
            RenderProps.setLineStyle(forceps, LineStyle.CYLINDER);

            getMechMod().addFrameMarker (p1);
            getMechMod().addFrameMarker (p2);
            getMechMod().addAxialSpring (forceps);
            
            this.registerForceps (forceps);
            forceps.setTightening (0);
            return forceps;

         } catch (Exception e) {
            e.printStackTrace();
         }
      }

      return null;
   }
   
   public Ligament legacyAddAttachmentPoint (String name, RigidBody rigidBody, int index) {
      if (rigidBody != null)
      {
         Ligament attachmentPoint = new Ligament ();
         attachmentPoint.setName (name);
         

         Vertex3d vertex = rigidBody.getMesh ().getVertex (index);

         if (vertex == null) 
            return null;

         FrameMarker p = new FrameMarker (vertex.getPosition ());
         p.setFrame (rigidBody);
         p.setFixed (true);
         getMechMod().addFrameMarker (p);
         
         this.setCorrectPointRadius(p);
         
         this.setCorrectLineRadius(attachmentPoint);
         
         Particle particle = new Particle (1.0, vertex.getWorldPoint ());
         particle.setDynamic (false);
         particle.setFixed (true);
         getMechMod().addParticle (particle);

         this.setCorrectPointRadius (particle);
         
         attachmentPoint.setPoints (particle, p);
         attachmentPoint.setRestLength (0.0);
         
         LigamentAxialMaterial mat = new LigamentAxialMaterial();
         
         mat.setElongStiffness (this.getAttachmentPointsStiffness ());
         
         attachmentPoint.setMaterial (mat);
         registerLigament(attachmentPoint, "Attachment Point");
 
         //getMechMod().addAxialSpring (attachmentPoint);
         
         return attachmentPoint;
      }

      return null;
   }
   
   private RigidBody legacyAddRigidBody (String bodyName, String fileName, double density) {
      try {
         //Setting rigidbody from OBJ mesh
         PolygonalMesh mesh = new PolygonalMesh (fileName);

         RigidBody rb = new RigidBody (bodyName);
         rb.setMesh (mesh, fileName);


         // Mechanical properties
         rb.setDensity (density);
         rb.setInertiaFromDensity (density);
         rb.setDynamic (true);
         
         
         getMechMod().addRigidBody (rb);

         RenderProps.setShading (rb, Shading.SMOOTH);
         RenderProps.setShininess (rb, 15.0);
         RenderProps.setFaceColor (rb, new Color(0xe3dac9));

         rb.setFrameDamping (getMechMod().getFrameDamping ());
         rb.setRotaryDamping (getMechMod().getRotaryDamping ());
         
         return rb;

      } catch (IOException e) {
         e.printStackTrace();
      }

      return null;
      
   }

}
