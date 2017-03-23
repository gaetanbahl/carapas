/**
 * Copyright (c) 2014, by the Authors: Antonio Sanchez (UBC)
 *
 * This software is freely available under a 2-clause BSD license. Please see
 * the LICENSE file in the ArtiSynth distribution directory for details.
 */
package maspack.geometry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;

import maspack.geometry.OBB.Method;
import maspack.matrix.Matrix3d;
import maspack.matrix.Plane;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.RotationMatrix3d;
import maspack.matrix.Vector2d;
import maspack.matrix.Vector3d;
import maspack.util.Logger;

/**
 * Constructive Solid Geometry tools
 * 
 * @author Antonio
 */
public class CSG {

   double myTol = -1;           // tolerance for considering "on" triangle
   static double EPS = 1e-12;   // machine tolerance

   static boolean verbose = false;

   public void setTolerance(double tol) {
      myTol = tol;
   }

   public double getTolerance() {
      return myTol;
   }

   //   public int getMaxRayCasts() {
   //      return myMaxRayCasts;
   //   }
   //
   //   public void setMaxRayCasts(int max) {
   //      myMaxRayCasts = max;
   //   }

   /**
    * Temporarily storing polygons before converting into faces
    * @author Antonio
    *
    */
   private static class Polygon implements Boundable {
      public Vertex3d[] vtxs;
      Plane plane;

      public Polygon(Vertex3d[] vtxs, Plane plane) {
         this.vtxs = Arrays.copyOf(vtxs, vtxs.length);
         this.plane = plane;
      }
      
      //      public Polygon(Vertex3d[] vtxs, Vector3d normal) {
      //         this.vtxs = Arrays.copyOf(vtxs, vtxs.length);
      //         plane = new Plane(normal, vtxs[0].getWorldPoint());
      //      }

      /**
       * Compute a point inside the polygon
       * @return
       */
      public Point3d getInteriorPoint() {
         Point3d out = new Point3d();

         if (vtxs.length > 3) {
            // find first convex vertex
            Vector3d vtx0 = vtxs[vtxs.length-2].getWorldPoint();
            Vector3d vtx1 = vtxs[vtxs.length-1].getWorldPoint();
            Vector3d vtx2;
            Vector3d v1 = new Vector3d();
            Vector3d v2 = new Vector3d();

            for (int i=0; i<vtxs.length; i++) {
               vtx2 = vtxs[i].getWorldPoint();

               v1.sub(vtx0, vtx1);
               v2.sub(vtx2, vtx1);
               v2.cross(v1);
               if (v2.dot(plane.normal) > 0) {
                  // get centroid
                  out.set(vtx0);
                  out.add(vtx1);
                  out.add(vtx2);
                  out.scale(1.0/3);

                  return out;
               }

               vtx0 = vtx1;
               vtx1 = vtx2;
            }
         }

         // default use first 3 vertices
         out.set(vtxs[0].getWorldPoint());
         out.add(vtxs[1].getWorldPoint());
         out.add(vtxs[2].getWorldPoint());
         out.scale(1.0/3);

         return out;
      }

      @Override
      public int numPoints() {
         return vtxs.length;
      }

      @Override
      public Point3d getPoint(int idx) {
         return vtxs[idx].getWorldPoint();
      }

      @Override
      public void computeCentroid(Vector3d centroid) {
         centroid.setZero();
         for (Vertex3d vtx : vtxs) {
            centroid.add(vtx.getWorldPoint());
         }
         centroid.scale(1.0/vtxs.length);
      }

      @Override
      public void updateBounds(Vector3d min, Vector3d max) {
         for (Vertex3d vtx : vtxs) {
            vtx.getWorldPoint().updateBounds(min, max);
         }
      }

      @Override
      public double computeCovariance(Matrix3d C) {
         double area = 0;
         C.setZero();

         Point3d p0 = vtxs[0].getWorldPoint();
         Point3d p2 = vtxs[1].getWorldPoint();
         Point3d p1 = null;
         
         int idx = 2;
         while (idx < vtxs.length) {
            p1 = p2;
            p2 = vtxs[idx].getWorldPoint();
            double a = CovarianceUtils.addTriangleCovariance (C, p0, p1, p2);
            area += a;
         }
         C.scale (1 / (12.0));

         // C is symmetric, so set symmetric components
         C.m10 = C.m01;
         C.m20 = C.m02;
         C.m21 = C.m12;

         return area;
      }

      public Polygon flipped() {
         Vertex3d[] fvtxs = new Vertex3d[vtxs.length];
         for (int i=0; i<vtxs.length; ++i) {
            fvtxs[i] = vtxs[vtxs.length-1-i];
         }
         Plane fplane = new Plane(plane);
         fplane.flip();
         return new Polygon(fvtxs, fplane);
      }

   }

   /**
    * Convert a list of polygons into a PolygonalMesh
    * @param polygons
    * @param mesh
    * @return
    */
   private static PolygonalMesh meshFromPolygons(ArrayList<Polygon> polygons, PolygonalMesh mesh) {
      if (mesh == null) {
         mesh = new PolygonalMesh();
      } else {
         mesh.clear();
      }

      HashMap<Vertex3d,Vertex3d> vtxMap = new HashMap<Vertex3d,Vertex3d>();

      for (Polygon poly : polygons) {

         Vertex3d[] nvtxs = new Vertex3d[poly.vtxs.length];

         int idx = 0;
         for (Vertex3d vtx : poly.vtxs) {
            Vertex3d nvtx = vtxMap.get(vtx);
            if (nvtx == null) {
               nvtx = mesh.addVertex(vtx.getWorldPoint());
               vtxMap.put(vtx, nvtx);
            }
            nvtxs[idx++] = nvtx;
         }
         mesh.addFace(nvtxs);

      }

      double parea = integrateArea(polygons);
      double marea = mesh.computeArea();
      double pvolume = integrateVolume(polygons);
      double mvolume = mesh.computeVolume();

      if (Math.abs(parea-marea) > 1e-12) {
         System.err.println("broken");
      }
      if (Math.abs(pvolume-mvolume) > 1e-12) {
         System.err.println("broken");
      }

      return mesh;
   }
   
   public static PolygonalMesh getIntersection(PolygonalMesh mesh1, PolygonalMesh mesh2) {
      double tol = Math.min(mesh1.getRadius()*EPS, mesh2.getRadius()*EPS);
      ArrayList<Polygon> polys = cheapIntersection(mesh1, mesh2, tol);
      mergeCoincidentVertices(polys, tol);
      PolygonalMesh out = meshFromPolygons(polys, null);
      MeshFactory.closeSeams(out);
      return out;
   }

   /**
    * Slices one mesh by the other, using a fast plane-cutting based technique
    */
   public static void sliceMeshes(PolygonalMesh mesh1, PolygonalMesh mesh2, 
      double tol, PolygonalMesh outMesh1, PolygonalMesh outMesh2) {

      BVTree bvt1 = mesh1.getBVTree();
      BVTree bvt2 = mesh2.getBVTree();

      HashMap<BVNode,ArrayList<Polygon>> polyMap1 = new HashMap<BVNode,ArrayList<Polygon>>();
      HashMap<BVNode,ArrayList<Polygon>> polyMap2 = new HashMap<BVNode,ArrayList<Polygon>>();

      doMeshSlicing(bvt1, polyMap1, bvt2, polyMap2, tol);

      ArrayList<Polygon> slice1 = new ArrayList<Polygon>();
      for (ArrayList<Polygon> polyList : polyMap1.values()) {
         slice1.addAll(polyList);
      }

      ArrayList<Polygon> slice2 = new ArrayList<Polygon>();
      for (ArrayList<Polygon> polyList : polyMap2.values()) {
         slice2.addAll(polyList);
      }

      outMesh1 = meshFromPolygons(slice1, outMesh1);
      outMesh2 = meshFromPolygons(slice2, outMesh2);

   }

   //   private static Vertex3d maxAngle(Vertex3d vtx0, Vertex3d vtx1, Vertex3d vtx2) {
   //      
   //      // determine minimum angle on face
   //      Vector3d v = new Vector3d();
   //      
   //      v.sub(vtx1.getPosition(), vtx0.getPosition());
   //      double a = v.norm();
   //      v.sub(vtx2.getPosition(), vtx1.getPosition());
   //      double b = v.norm();
   //      v.sub(vtx0.getPosition(), vtx2.getPosition());
   //      double c = v.norm();
   //      
   //      double t1 = cosLawAngle(a,b,c); // v1
   //      double t2 = cosLawAngle(b,c,a); // v2
   //      double t3 = cosLawAngle(c,a,b); // v0
   //      
   //      if (t1 > t2) {
   //         if (t1 > t3) {
   //            return vtx1;
   //         } else {
   //            return vtx0;
   //         }
   //      } else {
   //         if (t2 > t3) {
   //            return vtx2;
   //         }
   //      }
   //      return vtx0;
   //   }
   //   private static void edgeFlipSmallFaces(PolygonalMesh mesh, double minArea) {
   //      if (!mesh.isTriangular ()) {
   //         mesh.triangulate();
   //      }
   //      
   //      ArrayList<Face> toFlip = new ArrayList<Face>();
   //      for (Face f : mesh.getFaces ()) {
   //         if (f.computeArea () < minArea) {
   //            toFlip.add (f);
   //         }
   //      }
   //      
   //      for (Face f : toFlip) {
   //         
   //         HalfEdge he0 = f.he0;
   //         HalfEdge he1 = he0.next;
   //         HalfEdge he2 = he1.next;
   //         
   //         // find largest angle
   //         Vertex3d vtx0 = he0.head;
   //         Vertex3d vtx1 = he1.head;
   //         Vertex3d vtx2 = he2.head;
   //         Vertex3d splitVtx = maxAngle (vtx0, vtx1, vtx2);
   //         
   //         HalfEdge f0 = he1.opposite;
   //         HalfEdge f1 = he2.opposite;
   //         HalfEdge f2 = he0.opposite;
   //         
   //         if (mesh.removeFaceFast (f)) {
   //            if (splitVtx == vtx0) {
   //               if (f0 != null) {
   //                  Vertex3d vtxOpp = f0.next.head;
   //                  mesh.removeFaceFast (f0.face);
   //                  mesh.addFace (vtx0, vtx1, vtxOpp);
   //                  mesh.addFace (vtx0, vtxOpp, vtx2);
   //               }
   //            } else if (splitVtx == vtx1) {
   //               if (f1 != null) {
   //                  Vertex3d vtxOpp = f1.next.head;
   //                  mesh.removeFaceFast (f1.face);
   //                  mesh.addFace (vtx1, vtx2, vtxOpp);
   //                  mesh.addFace (vtx1, vtxOpp, vtx0);
   //               }
   //            } else if (splitVtx == vtx2) {
   //               if (f2 != null) {
   //                  Vertex3d vtxOpp = f2.next.head;
   //                  mesh.removeFaceFast (f2.face);
   //                  mesh.addFace (vtx2, vtx0, vtxOpp);
   //                  mesh.addFace (vtx2, vtxOpp, vtx1);
   //               }
   //            }
   //         }
   //         
   //      }
   //   }

   private static double minAngle(Vertex3d vtx0, Vertex3d vtx1, Vertex3d vtx2) {

      // determine minimum angle on face
      Vector3d v = new Vector3d();

      v.sub(vtx1.getPosition(), vtx0.getPosition());
      double a = v.norm();
      v.sub(vtx2.getPosition(), vtx1.getPosition());
      double b = v.norm();
      v.sub(vtx0.getPosition(), vtx2.getPosition());
      double c = v.norm();

      double t1 = cosLawAngle(a,b,c);
      double t2 = cosLawAngle(b,c,a);
      double t3 = cosLawAngle(c,a,b);

      double t = Math.min(t1, t2);
      t = Math.min(t,  t3);

      return t;
   }

   private static double cosLawAngle(double a, double b, double c) {
      double t = (a*a+b*b-c*c)/(2*a*b);
      if (t > 1) {
         t = 1;
      } else if (t < -1) {
         t = -1;
      }
      t = Math.acos(t);
      return t;
   }


   public static void sliceMesh(PolygonalMesh mesh, BVTree bvt, Plane plane, double baryTol, double normalTol) {
      sliceMesh(mesh, bvt, plane,  baryTol, normalTol, 0, 0);
   }

   public static void sliceMesh(PolygonalMesh mesh, BVTree bvt, Plane plane, double baryTol, double normalTol, 
      int newVtxFlag, int isectFlag) {

      if (!mesh.isTriangular()) {
         mesh.triangulate ();
      }

      if (bvt == null) {
         bvt = mesh.getBVTree();
      }

      ArrayList<BVNode> nodes = new ArrayList<BVNode>();
      bvt.intersectPlane (nodes, plane);

      HashMap<HalfEdge,Vertex3d> edgeSplit = new HashMap<HalfEdge,Vertex3d>();

      //XXX
      // mark edges to split
      for (BVNode node : nodes) {
         for (Boundable b : node.getElements()) {
            Face f = (Face)b;
            if ( Math.abs (f.getNormal().dot(plane.normal)) <= 1-normalTol  ) {

               // loop through edges, potentially mark a cut
               HalfEdge he0 = f.he0;
               HalfEdge he = he0;
               do {
                  if (he.isPrimary()) {
                     // find if edge cut by plane
                     Point3d hp = he.head.getPosition ();
                     Point3d tp = he.tail.getPosition ();
                     double dh = plane.distance (hp);
                     double dt = plane.distance (tp);
                     if (dh*dt < 0) { // opposite sides
                        double t = dh/(dh-dt);
                        if (t >= baryTol && t <= 1-baryTol ) {
                           Point3d pnt = new Point3d();
                           pnt.interpolate (hp, t, tp);

                           Vertex3d nvtx = mesh.addVertex (pnt);
                           nvtx.setFlag (isectFlag);
                           nvtx.setFlag (newVtxFlag);
                           edgeSplit.put (he, nvtx);
                           if (he.opposite != null) {
                              edgeSplit.put (he.opposite, nvtx);
                           }
                        }
                     }

                  }
                  he = he.next;
               } while (he != he0);

            } // plane not parallel to face
         } // boundable in node
      } // nodes

      int[] types = new int[7];

      // actually split faces
      for (BVNode node : nodes) {
         ArrayList<Boundable> boundablesToRemove = new ArrayList<Boundable>();
         ArrayList<Boundable> boundablesToAdd = new ArrayList<Boundable>();

         for (Boundable b : node.getElements()) {
            Face f = (Face)b;
            if ( Math.abs (f.getNormal().dot(plane.normal)) <= 1-normalTol  ) {

               // Get points along edge, and make sure none are lost
               HalfEdge he0 = f.he0;
               Vertex3d v0 = edgeSplit.get (he0); 
               HalfEdge he1 = he0.next;
               Vertex3d v1 = edgeSplit.get (he1);
               HalfEdge he2 = he1.next;
               Vertex3d v2 = edgeSplit.get (he2);

               if (v0 != null || v1 != null || v2 != null) {
                  mesh.removeFaceFast (f);
                  boundablesToRemove.add(f);
               }
               //               mesh.isManifold ();

               if ((v0 != null && v0.idx == 4421) || (v1 != null && v1.idx == 4421) || (v2 != null && v2.idx == 4421)
                  || he0.head.idx == 4421 || he1.head.idx == 4421 || he2.head.idx == 4421 )
               {
                  //                  System.out.println ("What's going on here?");
                  //                  mesh.findNonManifoldVertices ();
                  //                  System.out.println ("^ before");
               }
               if (v0 != null && v1 != null && v2 != null) {
                  // 3-split
                  Face nf;
                  nf = mesh.addFace (v0, he0.head, v1); 
                  boundablesToAdd.add (nf);
                  nf = mesh.addFace (v1, he1.head, v2);
                  boundablesToAdd.add (nf);
                  nf = mesh.addFace (v2, he2.head, v0);
                  boundablesToAdd.add (nf);
                  nf = mesh.addFace (v0, v1, v2);
                  boundablesToAdd.add (nf);
                  //                  mesh.isManifold ();
                  types[0]++;
               } else if (v0 != null && v1 != null) {

                  // 2-split
                  Face nf;
                  nf = mesh.addFace (v0, he0.head, v1); 
                  boundablesToAdd.add (nf);

                  // best next two faces (max min angle)
                  double s1 = minAngle (he2.head, v0, v1);
                  double s2 = minAngle(he1.head, he2.head, v1);
                  double smin = Math.min (s1, s2);

                  double t1 = minAngle (he1.head, he2.head, v0);
                  double t2 = minAngle(v1, he1.head, v0);
                  double tmin = Math.min (t1, t2);

                  if (smin > tmin) {
                     nf = mesh.addFace (he2.head, v0, v1);
                     boundablesToAdd.add (nf);
                     nf = mesh.addFace (he1.head, he2.head, v1);
                     boundablesToAdd.add (nf);
                  } else {
                     nf = mesh.addFace (he1.head, he2.head, v0);
                     boundablesToAdd.add (nf);
                     nf = mesh.addFace (v1, he1.head, v0);
                     boundablesToAdd.add (nf);
                  }
                  types[1]++;
                  //                  mesh.isManifold ();
               } else if (v0 != null && v2 != null) {
                  // 2-split
                  Face nf;
                  nf = mesh.addFace (v2, he2.head, v0); 
                  boundablesToAdd.add (nf);

                  // best next two faces (max min angle)
                  double s1 = minAngle (he1.head, v2, v0);
                  double s2 = minAngle(he0.head, he1.head, v0);
                  double smin = Math.min (s1, s2);

                  double t1 = minAngle (he0.head, he1.head, v2);
                  double t2 = minAngle(v0, he0.head, v2);
                  double tmin = Math.min (t1, t2);

                  if (smin > tmin) {
                     nf = mesh.addFace (he1.head, v2, v0);
                     boundablesToAdd.add (nf);
                     nf = mesh.addFace (he0.head, he1.head, v0);
                     boundablesToAdd.add (nf);
                  } else {
                     nf = mesh.addFace (he0.head, he1.head, v2);
                     boundablesToAdd.add (nf);
                     nf = mesh.addFace (v0, he0.head, v2);
                     boundablesToAdd.add (nf);
                  } 
                  types[2]++;
                  //                  mesh.isManifold ();
               } else if (v1 != null && v2 != null) {
                  // 2-split
                  Face nf;
                  nf = mesh.addFace (v1, he1.head, v2); 
                  boundablesToAdd.add (nf);

                  // best next two faces (max min angle)
                  double s1 = minAngle (he0.head, v1, v2);
                  double s2 = minAngle(he2.head, he0.head, v2);
                  double smin = Math.min (s1, s2);

                  double t1 = minAngle (he2.head, he0.head, v1);
                  double t2 = minAngle(v2, he2.head, v1);
                  double tmin = Math.min (t1, t2);

                  if (smin > tmin) {
                     nf = mesh.addFace (he0.head, v1, v2);
                     boundablesToAdd.add (nf);
                     nf = mesh.addFace (he2.head, he0.head, v2);
                     boundablesToAdd.add (nf);
                  } else {
                     nf = mesh.addFace (he2.head, he0.head, v1);
                     boundablesToAdd.add (nf);
                     nf = mesh.addFace (v2, he2.head, v1);
                     boundablesToAdd.add (nf);
                  }
                  types[3]++;
                  //                  mesh.isManifold ();
               } else if (v0 != null) {
                  Face nf = mesh.addFace (v0, he0.head, he1.head);
                  boundablesToAdd.add (nf);
                  nf = mesh.addFace (v0, he1.head, he2.head);
                  boundablesToAdd.add (nf);

                  // mark he1.head as falling on plane
                  he1.head.setFlag (isectFlag);
                  types[4]++;
                  //                  mesh.isManifold ();
               } else if (v1 != null) {
                  Face nf = mesh.addFace (v1, he1.head, he2.head);
                  boundablesToAdd.add (nf);
                  nf = mesh.addFace (v1, he2.head, he0.head);
                  boundablesToAdd.add (nf);

                  // mark he1.head as falling on plane
                  he2.head.setFlag (isectFlag);
                  types[5]++;
                  //                  mesh.isManifold ();
               } else if (v2 != null) {

                  Face nf = mesh.addFace (v2, he2.head, he0.head);
                  boundablesToAdd.add (nf);
                  nf = mesh.addFace (v2, he0.head, he1.head);
                  boundablesToAdd.add (nf);

                  // mark he1.head as falling on plane
                  he0.head.setFlag (isectFlag);
                  types[6]++;
                  //                  mesh.isManifold ();
               }

            } // plane not parallel to face
         } // boundable in node

         // update boundables
         if (boundablesToAdd.size () > 0 || boundablesToRemove.size () > 0) {
            ArrayList<Boundable> b = new ArrayList<Boundable>();
            b.addAll(Arrays.asList (node.getElements()));
            b.removeAll (boundablesToRemove);
            b.addAll (boundablesToAdd);

            node.setElements (b.toArray (new Boundable[b.size ()]));
         }
      } // nodes

      for (int i=0; i<types.length; i++) {
         System.out.println("Type " + i + ": " + types[i]);
      }

   }

   private static ArrayList<Polygon> cheapIntersection(PolygonalMesh mesh1, PolygonalMesh mesh2, 
      double tol) {

      BVTree bvt1 = mesh1.getBVTree();
      BVTree bvt2 = mesh2.getBVTree();
      return cheapIntersection(mesh1, bvt1, mesh2, bvt2, tol);
   }

   //   private static PolygonalMesh cheapIntersection(PolygonalMesh mesh1, BVTree bvt1, 
   //      PolygonalMesh mesh2, BVTree bvt2, 
   //      double tol, PolygonalMesh out) {
   //
   //      if (out == null) {
   //         out = new PolygonalMesh();
   //      }
   //
   //      ArrayList<Polygon> isect = cheapIntersection(mesh1, bvt1, mesh2, bvt2, tol);
   //      meshFromPolygons(isect, out);
   //
   //      return out;
   //   }

   private static ArrayList<Polygon> cheapIntersection(PolygonalMesh mesh1, BVTree bvt1,
      PolygonalMesh mesh2, BVTree bvt2, double tol) {
      
      if (tol < 0) {
         tol = Math.min(mesh1.computeRadius()*EPS, mesh2.computeRadius()*EPS);
      }

      HashMap<BVNode,ArrayList<Polygon>> polyMap1 = new HashMap<BVNode,ArrayList<Polygon>>();
      HashMap<BVNode,ArrayList<Polygon>> polyMap2 = new HashMap<BVNode,ArrayList<Polygon>>();

      doMeshSlicing(bvt1, polyMap1, bvt2, polyMap2, tol);

      BVFeatureQuery query = new BVFeatureQuery();
      // query.setMaxRayCasts(maxRayCasts);
      Point3d nearest = new Point3d();
      Vector2d uv = new Vector2d();

      // do lazy intersection
      ArrayList<Polygon> intersectionPolys = new ArrayList<Polygon>();

      // for every face in mesh1, check if inside mesh2
      //    inside -> keep, consistent coplanar -> keep, other -> discard
      // int nodeIdx = 1;
      for(ArrayList<Polygon> polyList : polyMap1.values()) {

         // int polyIdx = 1;
         for (Polygon poly : polyList) {

            // get a point in middle of face
            Point3d pnt = poly.getInteriorPoint();

            boolean inside = query.isInsideOrientedMesh(bvt2, pnt, tol);
            // boolean on = false;
            if (inside) {
               // check if "on"
               Face face = query.getFaceForInsideOrientedTest(nearest, uv);
               if (pnt.distanceSquared(nearest) <= tol) {
                  // on = true;
                  if (poly.plane.normal.dot(face.getNormal()) > 0.5) {
                     intersectionPolys.add(poly);
                  }
               } else {
                  // inside
                  intersectionPolys.add(poly);
               }
            }

            //            System.out.printf("Polygon %d:%d is ", nodeIdx, polyIdx);
            //            if (on) {
            //               System.out.printf("on");
            //            } else if (inside) {
            //               System.out.printf("in");
            //            } else {
            //               System.out.printf("out");
            //            }
            //            System.out.printf("\n");
            // polyIdx++;
         }  
         // nodeIdx++;
      } 

      // for every face in mesh2, check if inside mesh1
      //    inside -> keep, other -> discard
      // nodeIdx = 1;
      for(ArrayList<Polygon> polyList : polyMap2.values()) {

         // int polyIdx = 1;
         for (Polygon poly : polyList) {

            // get a point in middle of face
            Point3d pnt = poly.getInteriorPoint();
            
            boolean inside = query.isInsideOrientedMesh(bvt1, pnt, tol);
            if (inside) {
               // check if "on"
               query.getFaceForInsideOrientedTest(nearest, uv);
               if (pnt.distanceSquared(nearest) > tol) {
                  // inside, not on
                  intersectionPolys.add(poly);
               }
            }

            // polyIdx++;
         }  
         // nodeIdx++;
      } 

      return intersectionPolys;
   }
   
   private static void mergeCoincidentVertices(ArrayList<Polygon> polys, double tol) {
      BVTree bvtree = new OBBTree();
      bvtree.setMargin(tol);
      bvtree.build(polys);
      
      for (Polygon poly : polys) {
         for (Vertex3d vtx : poly.vtxs) {
            vtx.clearVisited();
         }
      }
      
      HashMap<Vertex3d, Vertex3d> vtxMap = new HashMap<>();
      HashMap<Vertex3d,HashSet<Vertex3d>> vtxSets = new HashMap<>();
      
      for (Polygon poly : polys) {
         for (Vertex3d vtx : poly.vtxs) {
            if (!vtx.isVisited()) {
               ArrayList<BVNode> nodes = new ArrayList<BVNode>();
               Point3d vtxp = vtx.getWorldPoint();
               bvtree.intersectPoint(nodes, vtxp);
               
               HashSet<Vertex3d> vtxSet = new HashSet<>();
               vtxSet.add(vtx);
               
               for (BVNode node : nodes) {
                  for (Boundable bound : node.getElements()) {
                     Polygon pp = (Polygon)bound;
                     for (Vertex3d vv : pp.vtxs) {
                        if (vv != vtx) {
                           Point3d vvp = vv.getWorldPoint();
                           if (vtxp.distance(vvp) <= tol) {
                              if (vv.isVisited()) {
                                 // merge group with current group
                                 Vertex3d vgroup = vtxMap.get(vv);
                                 if (vgroup != vtx) {
                                    HashSet<Vertex3d> other = vtxSets.get(vgroup);
                                    for (Vertex3d vo : other) {
                                       vtxMap.put(vo, vtx);
                                    }
                                    vtxSet.addAll(other);
                                    vtxSets.remove(vgroup);
                                 }
                                 
                              } else {
                                 // merge with current group
                                 vtxMap.put(vv, vtx);
                                 vtxSet.add(vv);
                                 vv.setVisited();
                              }
                           }
                        }
                     }
                  }
               }
               
               vtxSets.put(vtx, vtxSet);
               vtxMap.put(vtx, vtx);
               vtx.setVisited();
            }
         }
      }
      
      // go through and replace vertices in polygons
      HashSet<Polygon> toRemove = new HashSet<>();
      ArrayList<Polygon> toAdd = new ArrayList<>();
      
      for (Polygon poly : polys) {
         for (int i=0; i<poly.vtxs.length; ++i) {
            poly.vtxs[i] = vtxMap.get(poly.vtxs[i]);
         }
         
         // check if we need to split or remove polygon
         for (Vertex3d vtx : poly.vtxs) {
            vtx.clearVisited();
         }

         for (int i=0; i<poly.vtxs.length; ++i) {
            if (poly.vtxs[i].isVisited()) {
               // find out how far back vertex was visited
               int vstart = -1;
               for (int j=i; j-->0;) {
                  poly.vtxs[j].clearVisited();
                  if (poly.vtxs[j] == poly.vtxs[i]) {
                     vstart = j;
                     break;
                  }
               }

               // split off vstart -> i
               int nvtxsn = i-vstart-1;
               if (nvtxsn > 2) {
                  Vertex3d[] nvtxs = new Vertex3d[nvtxsn];
                  for (int j=0; j<nvtxsn; ++j) {
                     nvtxs[j] = poly.vtxs[j+vstart];
                  }
                  Polygon npoly = new Polygon(nvtxs, poly.plane); 
                  toAdd.add(npoly);
               }
               
               // remove vertices from what remains
               Vertex3d[] nvtxs = new Vertex3d[poly.vtxs.length-nvtxsn];
               for (int j=0; j<=vstart; ++j) {
                  nvtxs[j] = poly.vtxs[j];
               }
               int idx = vstart+1;
               for (int k=i+1; k<nvtxs.length; ++k) {
                  nvtxs[idx++] = poly.vtxs[k];
               }
               poly.vtxs = nvtxs;
               poly.vtxs[vstart].setVisited();
               i = vstart;
               
               if (poly.vtxs.length < 3) {
                  toRemove.add(poly);
                  break;
               }
            } else {
               poly.vtxs[i].setVisited();
            }
         }
      }
      
      if (toRemove.size() > 0) {
         polys.removeAll(toRemove);
      }
      
   }
   
   public static PolygonalMesh getSubtraction(PolygonalMesh mesh1, PolygonalMesh mesh2) {
      double tol = Math.min(mesh1.getRadius()*EPS, mesh2.getRadius()*EPS);
      ArrayList<Polygon> subtract = cheapSubtraction(mesh1, mesh1.getBVTree(), 
         mesh2, mesh2.getBVTree(), tol);
      mergeCoincidentVertices(subtract, tol);
      PolygonalMesh out = meshFromPolygons(subtract, null);
      MeshFactory.closeSeams(out);
      //      out.mergeCoplanarFaces(Math.cos(Math.toRadians(1))); // within 1 degree
      //      out.triangulate();
      return out;
   }
   
   public static PolygonalMesh getCheapSubtraction(PolygonalMesh mesh1, PolygonalMesh mesh2) {
      double tol = Math.min(mesh1.getRadius()*EPS, mesh2.getRadius()*EPS);
      ArrayList<Polygon> subtract = cheapSubtraction(mesh1, mesh1.getBVTree(), 
         mesh2, mesh2.getBVTree(), tol);
      mergeCoincidentVertices(subtract, tol);
      PolygonalMesh out = meshFromPolygons(subtract, null);
      // out.mergeCoplanarFaces(Math.cos(Math.toRadians(1)));
      // MeshFactory.closeSeams(out);
      return out;
   }
   
   private static ArrayList<Polygon> cheapSubtraction(PolygonalMesh mesh1, BVTree bvt1,
      PolygonalMesh mesh2, BVTree bvt2, double tol) {

      HashMap<BVNode,ArrayList<Polygon>> polyMap1 = new HashMap<BVNode,ArrayList<Polygon>>();
      HashMap<BVNode,ArrayList<Polygon>> polyMap2 = new HashMap<BVNode,ArrayList<Polygon>>();

      doMeshSlicing(bvt1, polyMap1, bvt2, polyMap2, tol);

      BVFeatureQuery query = new BVFeatureQuery();
      // query.setMaxRayCasts(maxRayCasts);
      Point3d nearest = new Point3d();
      Vector2d uv = new Vector2d();

      // do lazy subtraction
      ArrayList<Polygon> subtractionPolys = new ArrayList<Polygon>();

      // for every face in mesh1, check if inside mesh2
      //    inside -> discard, consistent coplanar -> discard, other -> keep
      // int nodeIdx = 1;
      for(ArrayList<Polygon> polyList : polyMap1.values()) {

         // int polyIdx = 1;
         for (Polygon poly : polyList) {

            // get a point in middle of face
            Point3d pnt = poly.getInteriorPoint();
            boolean inside = query.isInsideOrientedMesh(bvt2, pnt, tol);
            // boolean on = false;
            if (inside) {
               // check if "on"
               Face face = query.getFaceForInsideOrientedTest(nearest, uv);
               if (pnt.distanceSquared(nearest) <= tol) {
                  // on = true;
                  if (poly.plane.normal.dot(face.getNormal()) < 0.5) {
                     // inconsistent coplanar
                     subtractionPolys.add(poly);
                  }
               }
            } else {
               // outside
               subtractionPolys.add(poly);
            }

            //            System.out.printf("Polygon %d:%d is ", nodeIdx, polyIdx);
            //            if (on) {
            //               System.out.printf("on");
            //            } else if (inside) {
            //               System.out.printf("in");
            //            } else {
            //               System.out.printf("out");
            //            }
            //            System.out.printf("\n");
            // polyIdx++;
         }  
         // nodeIdx++;
      } 

      // for every face in mesh2, check if inside mesh1
      //    inside -> keep flipped, other -> discard
      // nodeIdx = 1;
      for(ArrayList<Polygon> polyList : polyMap2.values()) {

         // int polyIdx = 1;
         for (Polygon poly : polyList) {

            // get a point in middle of face
            Point3d pnt = poly.getInteriorPoint();
            boolean inside = query.isInsideOrientedMesh(bvt1, pnt, tol);
            if (inside) {
               // check if "on"
               query.getFaceForInsideOrientedTest(nearest, uv);
               if (pnt.distanceSquared(nearest) > tol) {
                  // inside, not on
                  subtractionPolys.add(poly.flipped());
               }
            }
            /// polyIdx++;
         }  
         // nodeIdx++;
      } 

      return subtractionPolys;
   }
   
   private static ArrayList<Polygon> cheapUnion(PolygonalMesh mesh1, BVTree bvt1,
      PolygonalMesh mesh2, BVTree bvt2, 
      double tol) {

      HashMap<BVNode,ArrayList<Polygon>> polyMap1 = new HashMap<BVNode,ArrayList<Polygon>>();
      HashMap<BVNode,ArrayList<Polygon>> polyMap2 = new HashMap<BVNode,ArrayList<Polygon>>();

      doMeshSlicing(bvt1, polyMap1, bvt2, polyMap2, tol);

      BVFeatureQuery query = new BVFeatureQuery();
      // query.setMaxRayCasts(maxRayCasts);
      Point3d nearest = new Point3d();
      Vector2d uv = new Vector2d();

      // do lazy union
      ArrayList<Polygon> unionPolys = new ArrayList<Polygon>();

      // for every face in mesh1, check if inside mesh2
      //    outside -> keep, consistent coplanar -> keep, other -> discard
      // int nodeIdx = 1;
      for(ArrayList<Polygon> polyList : polyMap1.values()) {
         // int polyIdx = 1;
         for (Polygon poly : polyList) {

            // get a point in middle of face
            Point3d pnt = poly.getInteriorPoint();
            boolean inside = query.isInsideOrientedMesh(bvt2, pnt, tol);
            // boolean on = false;
            if (inside) {
               // check if "on"
               Face face = query.getFaceForInsideOrientedTest(nearest, uv);
               if (pnt.distanceSquared(nearest) <= tol) {
                  // on = true;
                  if (poly.plane.normal.dot(face.getNormal()) > 0.5) {
                     unionPolys.add(poly);
                  }
               }
            } else {
               unionPolys.add(poly);
            }

            //            System.out.printf("Polygon %d:%d is ", nodeIdx, polyIdx);
            //            if (on) {
            //               System.out.printf("on");
            //            } else if (inside) {
            //               System.out.printf("in");
            //            } else {
            //               System.out.printf("out");
            //            }
            //            System.out.printf("\n");

            // polyIdx++;
         }  
         // nodeIdx++;
      } 

      // for every face in mesh2, check if inside mesh1
      //    outside -> keep, other -> discard
      // nodeIdx = 1;
      for(ArrayList<Polygon> polyList : polyMap2.values()) {

         // int polyIdx = 1;
         for (Polygon poly : polyList) {

            // get a point in middle of face
            Point3d pnt = poly.getInteriorPoint();

            //            InsideQuery res = query.isInsideOrOnMesh(mesh1, bvt1, pnt, tol);
            //            if (res == InsideQuery.UNSURE) {
            //               return null;
            //            } else if (res == InsideQuery.INSIDE) {
            //               intersectionPolys.add(poly);
            //            } 

            boolean inside = query.isInsideOrientedMesh(bvt1, pnt, tol);
            if (!inside) {
               unionPolys.add(poly);
            }

            // polyIdx++;
         }  
         // nodeIdx++;
      } 

      return unionPolys;
   }
   
   public static PolygonalMesh getUnion(PolygonalMesh mesh1, PolygonalMesh mesh2) {
      double tol = Math.min(mesh1.getRadius()*EPS, mesh2.getRadius()*EPS);
      ArrayList<Polygon> subtract = cheapUnion(mesh1, mesh1.getBVTree(), 
         mesh2, mesh2.getBVTree(), tol);
      mergeCoincidentVertices(subtract, tol);
      PolygonalMesh out = meshFromPolygons(subtract, null);
      MeshFactory.closeSeams(out);
      return out;
   }

   /**
    * Computes the intersection volume contained by two meshes
    */
   public static double computeIntersectionVolume(PolygonalMesh mesh1, PolygonalMesh mesh2, double tol) {

      ArrayList<Polygon> intersectionPolys = cheapIntersection(mesh1, mesh2, tol);

      // compute volume
      return integrateVolume(intersectionPolys);
   }

   /**
    * Computes the intersection volume contained by two meshes
    */
   public static double computeIntersectionVolume(PolygonalMesh mesh1, BVTree tree1, PolygonalMesh mesh2, 
      BVTree tree2, double tol) {

      ArrayList<Polygon> intersectionPolys = cheapIntersection(mesh1, tree1, mesh2, tree2, tol);
      // compute volume
      return integrateVolume(intersectionPolys);
   }

   private static double integrateArea(ArrayList<Polygon> polygons) {

      double area = 0;

      for (Polygon p : polygons) {
         if (!p.plane.normal.containsNaN()) {

            Vertex3d v0 = p.vtxs[0];
            Vertex3d v1 = p.vtxs[1];
            double d2x = v1.pnt.x - v0.pnt.x;
            double d2y = v1.pnt.y - v0.pnt.y;
            double d2z = v1.pnt.z - v0.pnt.z;

            for (int i=2; i<p.vtxs.length; i++) {

               Vertex3d v2 = p.vtxs[i];

               double d1x = d2x;
               double d1y = d2y;
               double d1z = d2z;

               d2x = v2.pnt.x - v0.pnt.x;
               d2y = v2.pnt.y - v0.pnt.y;
               d2z = v2.pnt.z - v0.pnt.z;

               double nx = d1y * d2z - d1z * d2y;
               double ny = d1z * d2x - d1x * d2z;
               double nz = d1x * d2y - d1y * d2x;

               area += Math.sqrt (nx*nx + ny*ny + nz*nz);
            }
         }

      }

      area = area/2;
      return area;
   }

   private static double integrateVolume(ArrayList<Polygon> polygons) {

      double vol = 0;

      for (Polygon poly : polygons) {

         Vector3d vtxBase = poly.vtxs[0].getWorldPoint();

         // compute projection direction
         Vector3d nrm = poly.plane.normal;
         double x = Math.abs(nrm.x);
         double y = Math.abs(nrm.y);
         double z = Math.abs(nrm.z);

         int c = (x >= y) ? ((x >= z) ? 0 : 2) : ((y >= z) ? 1 : 2);
         int a = (c + 1) % 3;
         int b = (c + 2) % 3;

         // inertial moments
         double I = 0, Ia = 0, Ib = 0;

         // walk around the face
         Vector3d v0;
         Vector3d v1 = poly.vtxs[poly.vtxs.length-1].getWorldPoint();
         for (Vertex3d vtx : poly.vtxs) {
            v0 = v1;
            v1 = vtx.getWorldPoint();

            double a0 = v0.get(a);
            double a1 = v1.get(a);
            double b0 = v0.get(b);
            double b1 = v1.get(b);

            double C0 = a0 + a1;
            double Ca = a1*C0 + a0*a0;
            double Cb = b1*(b0 + b1) + b0*b0;

            I += (b1-b0) * C0;
            Ia += (b1-b0) * Ca;
            Ib += (a1-a0) * Cb;
         }

         I /= 2.0;
         Ia /= 6.0;
         Ib /= -6.0;

         double d = -nrm.dot (vtxBase);

         double na = nrm.get(a);
         double nb = nrm.get(b);
         double nc = nrm.get(c);
         double ncinv = 1.0 / nc;   // for fast division

         if (a == 0) {
            vol += ncinv * na * Ia;
         } else if (b == 0) {
            vol += ncinv * nb * Ib;
         } else {
            vol -= ((d * I + na * Ia + nb * Ib) * ncinv);
         }
      }

      return vol;

   }

   private static void clipPolygons(ArrayList<Polygon> polys, Plane plane, 
      double tol, ArrayList<Polygon> front, ArrayList<Polygon> back, 
      ArrayList<Polygon> coplanarFront, ArrayList<Polygon> coplanarBack) {

      final int COPLANAR = 0;
      final int FRONT = 1;
      final int BACK = 2;
      final int SPANNING = 3;

      for (Polygon poly : polys) {

         int pType = 0;
         int nVerts = poly.vtxs.length;
         int[] vTypes = new int[nVerts];

         // classify each vertex and the polygon
         for (int i=0; i<nVerts; i++) {
            Point3d pnt = poly.vtxs[i].getWorldPoint();
            double t = plane.distance(pnt);
            if (t < -tol) {
               vTypes[i] = BACK;
            } else if (t > tol) {
               vTypes[i] = FRONT;
            } else {
               vTypes[i] = COPLANAR;
            }
            pType |= vTypes[i];
         }

         // divide polygon by either placing it in the correct bin, or splitting
         switch(pType) {
            case COPLANAR: {
               if (plane.normal.dot(poly.plane.normal) > 0) {
                  coplanarFront.add(poly);
               } else {
                  coplanarBack.add(poly);
               }
               break;
            }
            case FRONT: {
               front.add(poly);
               break;
            }
            case BACK: {
               back.add(poly);
               break;
            }
            case SPANNING: {
               ArrayList<Vertex3d> f = new ArrayList<Vertex3d>();
               ArrayList<Vertex3d> b = new ArrayList<Vertex3d>();

               for (int i=0; i<poly.vtxs.length; i++) {
                  int j = (i+1)%poly.vtxs.length;
                  int ti = vTypes[i];
                  int tj = vTypes[j];

                  Vertex3d vi = poly.vtxs[i];
                  Vertex3d vj = poly.vtxs[j];

                  // front or back
                  if (ti != BACK) {
                     f.add(vi);
                  }
                  if (ti != FRONT) {
                     if (ti != BACK) {
                        b.add(vi);  // was clone
                     } else {
                        b.add(vi);
                     }
                  }

                  if ((ti | tj) == SPANNING) {
                     Vector3d diff = new Vector3d(vj.getWorldPoint());
                     diff.sub(vi.getWorldPoint());

                     double t = -plane.distance(vi.getWorldPoint()) / plane.normal.dot(diff);
                     
                     Vertex3d v = null;
                     if (t > -EPS && t < EPS) {
                        v = vi;
                     } else if (t > 1-EPS && t<1+EPS) {
                        v = vj;
                     } else {
                        Point3d pnt = new Point3d();
                        pnt.interpolate(vi.getWorldPoint(), t, vj.getWorldPoint());
                        v = new Vertex3d(pnt);
                     }

                     f.add(v);
                     b.add(v); //v.clone());
                  }

               }
               if (f.size() >= 3) {
                  front.add(new Polygon(f.toArray(new Vertex3d[f.size()]), poly.plane));
               }
               if (b.size() >= 3) {
                  back.add(new Polygon(b.toArray(new Vertex3d[b.size()]), poly.plane));
               }
               break;
            }
         }
      }

   }

   //   private static void printPoly(Polygon poly, String idStr) {
   //      if (verbose) {
   //         System.out.println(idStr);
   //         printPoly(poly);
   //      }
   //   }
   //   
   //   private static void printPoly(Polygon poly) {
   //      for (Vertex3d vtx : poly.vtxs) {
   //         System.out.print("  (" + vtx.getPosition().toString("%.2g") + ")");
   //      }
   //      System.out.println();
   //   }
   //   
   //   private static void printPolys(ArrayList<Polygon> polys, String idStr) {
   //      if (verbose) {
   //         System.out.println(idStr);
   //         for (Polygon poly : polys) {
   //            printPoly(poly);
   //         }
   //      }
   //   }

   private static void clipPolygons(ArrayList<Polygon> polys, ArrayList<Polygon> cutters, double tol, 
      ArrayList<Polygon> out) {

      ArrayList<Polygon> tmp = new ArrayList<Polygon>(polys);

      for (Polygon knife : cutters) {

         ArrayList<Polygon> sliced = new ArrayList<Polygon>();
         ArrayList<Polygon> coplanar = new ArrayList<Polygon>();

         //         printPolys(tmp, "In:");
         //         printPoly(knife, "Knife:");

         clipPolygons(tmp, knife.plane, tol, sliced, sliced, coplanar, coplanar);
         //       printPolys(sliced, "Sliced:");

         if (coplanar.size() > 0) {

            Vector3d dir = new Vector3d();
            int nVerts = knife.vtxs.length;
            Vertex3d vtx0 = knife.vtxs[nVerts-1];
            Vertex3d vtx1;

            for (int i=0; i<nVerts; i++) {
               vtx1 = knife.vtxs[i];
               dir.sub(vtx0.getWorldPoint(), vtx1.getWorldPoint());
               dir.cross(knife.plane.normal);

               ArrayList<Polygon> csliced = new ArrayList<Polygon>();
               double dn = dir.norm();
               if (dn > 0) {
                  dir.scale(1.0/dn);
                  Plane cplane = new Plane(dir, vtx0.getWorldPoint());
                  clipPolygons(coplanar, cplane, tol, csliced, csliced, csliced, csliced);

               }
               vtx0 = vtx1;

               coplanar = csliced;
            }
            // printPolys(coplanar,"Coplanar sliced:");
            sliced.addAll(coplanar);
         }

         tmp = sliced;
      }
      out.addAll(tmp);

   }

   private static void doMeshSlicing(BVTree bvt1, HashMap<BVNode,ArrayList<Polygon>> polyMap1,
      BVTree bvt2, HashMap<BVNode,ArrayList<Polygon>> polyMap2, double tol) {
      
      Logger logger = Logger.getSystemLogger();

      // need to add all leaves, not just intersecting ones
      ArrayList<BVNode> nodes1 = bvt1.getLeafNodes();
      ArrayList<BVNode> nodes2 = bvt2.getLeafNodes();
      HashMap<BVNode,Double> areaMap1 = new HashMap<BVNode,Double>();
      HashMap<BVNode,Double> areaMap2 = new HashMap<BVNode,Double>();

      for (BVNode node : nodes1) {
         ArrayList<Polygon> polys = new ArrayList<Polygon>();            
         Boundable[] elems = node.getElements();
         double area = 0;
         for (int j=0; j<node.getNumElements(); j++) {
            if (elems[j] instanceof Face) {
               Face face = (Face)elems[j];
               Vertex3d[] vtxs = face.getVertices();
               Polygon poly = new Polygon(vtxs, new Plane(face.getNormal(), vtxs[0].getWorldPoint()));
               polys.add(poly);
               area += face.computeArea();
            }
         }
         areaMap1.put(node, area);
         polyMap1.put(node, polys);
      }

      for (BVNode node : nodes2) {
         ArrayList<Polygon> polys = new ArrayList<Polygon>();            
         Boundable[] elems = node.getElements();
         double area = 0;
         for (int j=0; j<node.getNumElements(); j++) {
            if (elems[j] instanceof Face) {
               Face face = (Face)elems[j];
               Vertex3d[] vtxs = face.getVertices();
               Polygon poly = new Polygon(vtxs, new Plane(face.getNormal(), vtxs[0].getWorldPoint()));
               polys.add(poly);
               area += face.computeArea();
            }
         }
         areaMap2.put(node, area);
         polyMap2.put(node, polys);
      }

      nodes1 = new ArrayList<BVNode>();
      nodes2 = new ArrayList<BVNode>();

      bvt1.intersectTree(nodes1, nodes2, bvt2);

      int nNodes = nodes1.size();
      for (int i=0; i<nNodes; i++) {
         BVNode node1 = nodes1.get(i);
         BVNode node2 = nodes2.get(i);

         // get polygons inside node 1
         ArrayList<Polygon> polys1 = polyMap1.get(node1);

         // get polygons inside node 2
         ArrayList<Polygon> polys2 = polyMap2.get(node2);

         ArrayList<Polygon> clipped1 = new ArrayList<Polygon>();
         ArrayList<Polygon> clipped2 = new ArrayList<Polygon>();

         // cut faces in 1 by planes in 2
         clipPolygons(polys1, polys2, tol, clipped1);
         polyMap1.put(node1, clipped1);

         double area1a = integrateArea(polys1);
         double area1b = integrateArea(clipped1);
         double area1c = areaMap1.get(node1);
         if (Math.abs(area1a-area1b) > tol || Math.abs(area1a-area1c) > tol) {
            logger.debug("Warning: area mismatch, " + area1a + " vs " + area1b + " vs " + area1c);
         }

         // cut faces in 2 by planes in 1
         clipPolygons(polys2, polys1, tol, clipped2);
         //clipPolygons(polys2, clipped1, tol, clipped2);
         polyMap2.put(node2, clipped2);
         double area2a = integrateArea(polys2);
         double area2b = integrateArea(clipped2);
         double area2c = areaMap2.get(node2);
         if (Math.abs(area2a-area2b) > tol || Math.abs(area2a-area2c) > tol) {
            logger.debug("Warning: area mismatch, " + area2a + " vs " + area2b + " vs " + area2c);
         }

         if (verbose) {
            logger.info("Iter: " + i + ", ("  + 
               polys1.size() + ", " + polys2.size() + ") -> (" + 
               clipped1.size() + ", " + clipped2.size() + ")");
         }

         //if (clipped1.size() > polys1.size() || clipped2.size() > polys2.size()) {
         //   System.out.println("Sliced: " + (i+1) + "/" + nNodes + " (" + clipped1.size() + ", " + clipped2.size() + ")");
         //}
      }

   }

   /**
    * Computes Dice coefficient between two meshes.  The tolerance is
    *  used for robustness in "isInside" tests
    */
   public static double computeDice(PolygonalMesh mesh1, PolygonalMesh mesh2, double tol) {

      double v1 = mesh1.computeVolume();
      double v2 = mesh2.computeVolume();

      double vi = computeIntersectionVolume(mesh1, mesh2, tol);
      return (2*vi/(v1+v2));

   }

   /**
    * Computes Dice coefficient between two meshes.  The tolerance is
    *  used for robustness in "isInside" tests
    */
   public static double computeDice(PolygonalMesh mesh1, BVTree tree1, PolygonalMesh mesh2, BVTree tree2, double tol) {

      double v1 = mesh1.computeVolume();
      double v2 = mesh2.computeVolume();

      double vi = computeIntersectionVolume(mesh1, tree1, mesh2, tree2, tol);
      return (2*vi/(v1+v2));

   }

   /**
    * Computes the intersection volume between two meshes
    */
   public double computeIntersectionVolume(PolygonalMesh mesh1, PolygonalMesh mesh2) {
      return computeIntersectionVolume(mesh1, mesh2, myTol);
   }

   /**
    * Computes Dice coefficient between two meshes
    */
   public double computeDice(PolygonalMesh mesh1, PolygonalMesh mesh2) {
      return computeDice(mesh1, mesh2, myTol);
   }
  
   private static void doSphereTest(double r1, double r2, double d) {
      // test with spheres

      double svol1 = 4*Math.PI*(r1*r1*r1)/3;
      double svol2 = 4*Math.PI*(r2*r2*r2)/3;
      double svoli;
      if (d > r1 + r2) {
         svoli = 0;
      } else if (d == 0) {
         double minr = Math.min(r1, r2);
         svoli = 4*Math.PI*minr*minr*minr/3;
      } else {
         svoli = Math.PI/12/d*(r1+r2-d)*(r1+r2-d)*(d*d+2*d*r1-3*r1*r1+2*d*r2-3*r2*r2+6*r1*r2);
      }
      double sdice = 2*svoli/(svol1+svol2);

      PolygonalMesh s1 = MeshFactory.createOctahedralSphere(r1, 7);
      PolygonalMesh s2 = MeshFactory.createOctahedralSphere(r2, 7);
      s2.translate(new Vector3d(d,0,0));

      double mvol1 = s1.computeVolume();
      double mvol2 = s2.computeVolume();
      double mvoli = computeIntersectionVolume(s1, s2, 1e-15);
      double mdice = 2*mvoli/(mvol1+mvol2);

      System.out.println("Sphere test:");
      System.out.println("   Ideal dice: " + sdice);
      System.out.println("   Mesh dice: " + mdice);
   }

   private static void doCubeTest(Point3d dist) {

      PolygonalMesh m1 = MeshFactory.createQuadBox(1, 1, 1);
      PolygonalMesh m2 = MeshFactory.createQuadBox(1, 1, 1);
      m2.translate(dist);

      m1.triangulate();
      m2.triangulate();

      double mvol1 = m1.computeVolume();
      double mvol2 = m2.computeVolume();
      double mvoli = computeIntersectionVolume(m1, m2, 1e-12);
      double mdice = 2*mvoli/(mvol1+mvol2);
      System.out.println("Cube test:");
      System.out.println("   Mesh dice: " + mdice);
   }

   //   private static void printNode(OBB node, int depth) {
   //
   //      String space = "";
   //      for (int i=0; i<depth; i++) {
   //         space = space + "    ";
   //         System.out.print("----");
   //      }
   //
   //      Point3d center = new Point3d();
   //      node.getCenter(center);
   //      System.out.println("c: " + center.toString("%.3f"));
   //      System.out.println(space + "hw: " + node.getHalfWidths().toString("%.3f"));
   //
   //      BVNode child = node.getFirstChild();
   //      while (child != null) {
   //         printNode((OBB)child, depth+1);
   //         child = child.getNext();
   //      }
   //
   //   }

   //   private static void printTree(OBBTree tree) {
   //
   //      printNode(tree.getRoot(), 0);
   //
   //   }

   private static void doIntersectionTest(String fn1, String fn2) {

      try {
         PolygonalMesh m1 = new PolygonalMesh(fn1);
         PolygonalMesh m2 = new PolygonalMesh(fn2);

         OBBTree tree1 = new OBBTree(m1, Method.Covariance, 1, 1e-10);
         OBBTree tree2 = new OBBTree(m2, Method.Covariance, 1, 1e-10);

         // printTree((OBBTree)tree1);
         // printTree((OBBTree)tree2);

         ArrayList<BVNode> nodes1 = new ArrayList<BVNode>();
         ArrayList<BVNode> nodes2 = new ArrayList<BVNode>();
         tree1.intersectTree(nodes1, nodes2, tree2);
         System.out.println("Intersecting BVs: " + nodes1.size());

         ArrayList<Polygon> isect = cheapIntersection(m1, tree1, m2, tree2, 1E-10);
         System.out.println("Intersection size: " + isect.size());

      } catch (Exception e) {
         e.printStackTrace();
      }
   }

   private static void doDiceTest(String fn1, String fn2) {

      try {
         PolygonalMesh m1 = new PolygonalMesh(fn1);
         PolygonalMesh m2 = new PolygonalMesh(fn2);

         OBBTree tree1 = new OBBTree(m1, Method.Covariance, 1, 1e-10);
         OBBTree tree2 = new OBBTree(m2, Method.Covariance, 1, 1e-10);

         //         AABBTree tree1 = new AABBTree(m1, 1, 1e-10);
         //         AABBTree tree2 = new AABBTree(m2, 1, 1e-10);

         ArrayList<BVNode> nodes1 = tree1.getLeafNodes();
         ArrayList<BVNode> nodes2 = tree2.getLeafNodes();

         System.out.println("# Tree 1 has " + nodes1.size() + " leaves");
         System.out.println("# Tree 2 has " + nodes2.size() + " leaves");

         // printTree((OBBTree)tree1);
         // printTree((OBBTree)tree2);

         nodes1.clear();
         nodes2.clear();
         tree1.intersectTree(nodes1, nodes2, tree2);

         for (int i=0; i<nodes1.size(); i++) {
            BVNode node1 = nodes1.get(i);
            BVNode node2 = nodes2.get(i);

            System.out.printf("Intersection %d: \n", i);

            System.out.printf("   Node 1: ");
            for (int j=0; j<node1.getNumElements(); j++) {
               Boundable elem = node1.myElements[j];
               Face face = (Face)elem;

               System.out.printf(" (");
               for (Vertex3d v : face.getVertices()) {
                  System.out.printf(" %d", v.idx);
               }
               System.out.printf(" ) ");
            }
            System.out.printf("\n");

            System.out.printf("   Node 2: ");
            for (int j=0; j<node2.getNumElements(); j++) {
               Boundable elem = node2.myElements[j];
               Face face = (Face)elem;

               System.out.printf(" (");
               for (Vertex3d v : face.getVertices()) {
                  System.out.printf(" %d", v.idx);
               }
               System.out.printf(" ) ");
            }
            System.out.printf("\n");
         }

         double d = computeDice(m1, tree1, m2, tree2, 1e-10);
         System.out.println("# Dice: " + d);

      } catch (Exception e) {
         e.printStackTrace();
      }


   }

   private static void doObbTest() {

      OBB obb1 = new OBB();
      RigidTransform3d trans = new RigidTransform3d(new Point3d(0.5, -0.25, -0.25),
         new RotationMatrix3d(0.81649658092772603, 1.1102230246251565e-016, 0.57735026918962573, 0.40824829046386307, -0.70710678118654757, -0.57735026918962573, 0.40824829046386302, 0.70710678118654746, -0.57735026918962573));
      obb1.setTransform(trans);
      obb1.myHalfWidths.set(0.61237243579579459, 0.70710678128654769, 1.0000004163336343e-010);

      OBB obb2 = new OBB();
      trans = new RigidTransform3d(new Point3d( -0.099999999999999978, -0.25000000000000011, -0.24999999999999994),
         new RotationMatrix3d(0.81649658092772603, -1.1102230246251565e-016, -0.57735026918962573, -0.40824829046386307, -0.70710678118654757, -0.57735026918962573, -0.40824829046386302, 0.70710678118654746, -0.57735026918962573));
      obb2.setTransform(trans);
      obb2.myHalfWidths.set(0.61237243579579459, 0.70710678128654758, 1.0000005551115123e-010);

      boolean disjoint = BVBoxNodeTester.isDisjoint(obb1, obb2);
      System.out.printf("%s\n", disjoint ? "no" : "yes");
   }

   public static void main(String[] args) {

      doSphereTest(1, 1, 1);
      doCubeTest(new Point3d(0.75, 0.75, 0.75));

      doObbTest();

      if (args.length == 2) {
         doIntersectionTest(args[0], args[1]);
         doDiceTest(args[0], args[1]);
      }

   }


}
