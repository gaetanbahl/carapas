/**
 * Copyright (c) 2014, by the Authors: John E Lloyd (UBC)
 *
 * This software is freely available under a 2-clause BSD license. Please see
 * the LICENSE file in the ArtiSynth distribution directory for details.
 */
package maspack.render;

import java.awt.Color;

import javax.media.opengl.GL2;

public class Material {
   private float[] ambient;
   private float[] specular;
   private float[] diffuse;
   private float[] emission;
   private float[] temp;
   private float shininess;
   private float diffuseAmbienceFactor; // scale of diffuse

   public static final float[] default_ambient = {0.1f, 0.1f, 0.1f, 1f};
   public static final float[] default_specular = {0.1f, 0.1f, 0.1f, 1f};
   public static final float[] default_diffuse = {0.8f, 0.8f, 0.8f, 1f};
   public static final float[] default_emission = {0f, 0f, 0f, 1f};

   public static final int BLACK = 0;
   public static final int WHITE = 1;
   public static final int RED = 2;
   public static final int BLUE = 3;
   public static final int GREEN = 4;
   public static final int CYAN = 5;
   public static final int MAGENTA = 6;
   public static final int YELLOW = 7;
   public static final int GOLD = 8;
   public static final int GRAY = 9;
   public static final int SILVER = 10;

   // static public int rgbPack (double r, double g, double b)
   // {
   // int ri = Math.min(Math.max(0,r*255),255);
   // int gi = Math.min(Math.max(0,g*255),255);
   // int bi = Math.min(Math.max(0,b*255),255);
   // return (((bi)<<16) + ((gi)<<8) + (ri));
   // }

   // static public int rgbaPack (double r, double g, double b, double a)
   // {
   // int ri = Math.min(Math.max(0,r*255),255);
   // int gi = Math.min(Math.max(0,g*255),255);
   // int bi = Math.min(Math.max(0,b*255),255);
   // int ai = Math.min(Math.max(0,a*255),255);
   // return (((ai)<<24) + ((bi)<<16) + ((gi)<<8) + (ri));
   // }

   // static public void rgbUnpack (float[] vec, int rgb)
   // {
   // vec[0] = ((rgb) & 0xff)/255f;
   // vec[1] = ((rgb>>8) & 0xff)/255f;
   // vec[2] = ((rgb>>16) & 0xff)/255f;
   // }

   // static public void rgbaUnpack (float[] vec, int rgba)
   // {
   // vec[0] = ((rgb) & 0xff)/255f;
   // vec[1] = ((rgb>>8) & 0xff)/255f;
   // vec[2] = ((rgb>>16) & 0xff)/255f;
   // }

   public Material() {
      ambient = new float[4];
      specular = new float[4];
      diffuse = new float[4];
      emission = new float[4];
      temp = new float[4];
      setDefaults();
   }
   
   private void setDefaults() {
      setAmbient(default_ambient);
      setSpecular(default_specular);
      setDiffuse(default_diffuse);
      setEmission(default_emission);
   }

   public Material (Material mat) {
      this();
      set (mat);
   }

   public Material (float[] diff, float[] amb, float[] spec, float[] em, float shin, float diffuseAmbience) {
      this();
      set (diff, amb, spec, em, shin, diffuseAmbience);
   }
   
   public Material (Color diff, Color amb, Color spec, Color em, float shin, float diffuseAmbience) {
      this();
      set (diff, amb, spec, em, shin, diffuseAmbience);
   }
   
   
   public void set (float[] diff, float[] amb, float[] spec, float[] em, float shin, float diffuseAmbience) {
      setDiffuse (diff);
      setAmbient (amb);
      setSpecular (spec);
      setEmission(em);
      setShininess (shin);
      setAmbienceCoefficient(diffuseAmbience);
   }
   
   public void set (Color diff, Color amb, Color spec, Color em, float shin, float diffuseAmbience) {
      setDiffuse (diff);
      setAmbient (amb);
      setSpecular (spec);
      setEmission(em);
      setShininess (shin);
      setAmbienceCoefficient(diffuseAmbience);
   }

   public void set (Material mat) {
      set (mat.diffuse, mat.ambient, mat.specular, mat.emission, mat.shininess, mat.diffuseAmbienceFactor);
   }

   public void setAmbient (float r, float g, float b, float a) {
      ambient[0] = r;
      ambient[1] = g;
      ambient[2] = b;
      ambient[3] = a;
   }

   public void setAmbient (Color c) {
      c.getComponents (ambient);
   }
   
   public void setAmbient (float[] amb) {
      ambient[0] = amb[0];
      ambient[1] = amb[1];
      ambient[2] = amb[2];
      if (amb.length > 3) {
         ambient[3] = amb[3];
      }
      
   }

   // public void setAmbient (long rgba)
   // {
   // unpack (ambient, rgba);
   // }

   public float[] getAmbient() {
      return ambient;
   }

   public void setShininess (float s) {
      if (s < 0) {
         s = 0;
      }
      else if (s > 128) {
         s = 128;
      }
      shininess = s;
   }

   public float getShininess() {
      return shininess;
   }

   public void setSpecular (float r, float g, float b, float a) {
      specular[0] = r;
      specular[1] = g;
      specular[2] = b;
      specular[3] = a;
   }
   
   public void setSpecular(Color c) {
      c.getComponents (specular);
   }

   public void setSpecular (float[] spec) {
      specular[0] = spec[0];
      specular[1] = spec[1];
      specular[2] = spec[2];
      if (spec.length > 3) {
         specular[3] = spec[3];
      }
   }

   public float[] getSpecular() {
      return specular;
   }

   public void setDiffuse (float r, float g, float b, float a) {
      diffuse[0] = r;
      diffuse[1] = g;
      diffuse[2] = b;
      diffuse[3] = a;
   }

   public void setDiffuse (Color c) {
      float[] diff = new float[4];
      c.getComponents (diff);
      diffuse[0] = diff[0];
      diffuse[1] = diff[1];
      diffuse[2] = diff[2];
      diffuse[3] = diff[3];

   }

   public void setDiffuse (float[] diff) {
      diffuse[0] = diff[0];
      diffuse[1] = diff[1];
      diffuse[2] = diff[2];
      if (diff.length > 3) {
         diffuse[3] = diff[3];
      }
   }

   public float[] getDiffuse() {
      return diffuse;
   }

   public void setEmission(float r, float g, float b, float a) {
      emission[0] = r;
      emission[1] = g;
      emission[2] = b;
      emission[3] = a;
   }
   
   public void setEmission (Color c) {
      c.getComponents (emission);
   }
   
   public void setEmission(float[] em) {
      emission[0] = em[0];
      emission[1] = em[1];
      emission[2] = em[2];
      if (em.length > 3) {
         emission[3] = em[3];
      }
   }
   
   public float[] getEmission() {
      return emission;
   }
   
   public void apply (GL2 gl) {
      apply (gl, GL2.GL_FRONT_AND_BACK, null);
   }

   public void apply (GL2 gl, float[] diffuseOverride) {
      apply (gl, GL2.GL_FRONT_AND_BACK, diffuseOverride);
   }

   public void apply (GL2 gl, int sides) {
      apply (gl, sides, null);
   }

   public void apply (GL2 gl, int sides, float[] diffuseOverride) {
      
      float[] diff = diffuse;
      
      gl.glMaterialfv(sides, GL2.GL_EMISSION, emission, 0);
      gl.glMaterialf (sides, GL2.GL_SHININESS, shininess);
      gl.glMaterialfv (sides, GL2.GL_SPECULAR, specular, 0);
      if (diffuseOverride != null) {
         temp[0] = diffuseOverride[0];
         temp[1] = diffuseOverride[1];
         temp[2] = diffuseOverride[2];
         temp[3] = diffuse[3];
         diff = temp;
      }
      gl.glMaterialfv (sides, GL2.GL_DIFFUSE, diff, 0);
      // mix ambience and diffuse
      float a1 = diffuseAmbienceFactor;
      float a0 = 1-a1; 
      temp[0] = ambient[0]*a0 + diff[0]*a1;
      temp[1] = ambient[1]*a0 + diff[1]*a1;
      temp[2] = ambient[2]*a0 + diff[2]*a1;
      temp[3] = 1.0f;
      gl.glMaterialfv (sides, GL2.GL_AMBIENT, temp, 0);
   }

   private String floatArrayToString (float[] array) {
      return array[0] + " " + array[1] + " " + array[2] + " " + array[3];
   }

   public String toString() {
      String s = "";

      s += floatArrayToString (ambient) + "\n";
      s += floatArrayToString (specular) + "\n";
      s += floatArrayToString (diffuse) + "\n";
      s += floatArrayToString (emission) + "\n";
      s += shininess;
      return s;
   }

   public void setAlpha (double a) {
      //      ambient[3] = (float)a;
      //      specular[3] = (float)a;
      //      emission[3] = (float)a;
      // According to OpenGL reference, only diffuse alpha
      // is used in light model equation
      diffuse[3] = (float)a;
   }

   public boolean isTranslucent() {
      // return (ambient[3] != 1.0f || specular[3] != 1.0f 
      //   || diffuse[3] != 1.0f || emission[3] != 1.0f);
      
      // According to OpenGL reference, only diffuse alpha
      // used in light model equation
      return (diffuse[3] != 1.0f);
   }

   public static Material createDiffuse (
      float r, float g, float b, float a, float shine) {
      Material mat = new Material();
      mat.setAmbient (default_ambient);
      mat.setShininess (shine);
      mat.setSpecular (default_specular);
      mat.setEmission(default_emission);
      mat.setDiffuse (r, g, b, 1f);
      mat.setAlpha (a);
      return mat;
   }

   public static Material createDiffuse (float[] rgba, float shine) {
      return createDiffuse (rgba[0], rgba[1], rgba[2], rgba[3], shine);
   }

   public static Material createDiffuse (float[] rgb, float alpha, float shine) {
      return createDiffuse (rgb[0], rgb[1], rgb[2], alpha, shine);
   }

   public static Material createDiffuse (Color c, float shine) {
      float[] rgba = new float[4];
      c.getComponents (rgba);
      return createDiffuse (rgba, shine);
   }

   public static Material createDiffuse (Color c, float alpha, float shine) {
      float[] rgb = new float[4];
      c.getRGBComponents (rgb);
      return createDiffuse (rgb, alpha, shine);
   }

   public static Material createSpecial (int code) {
      switch (code) {
         case WHITE: {
            return createDiffuse (1f, 1f, 1f, 1f, 64f);
         }
         case RED: {
            return createDiffuse (1f, 0f, 0f, 1f, 64f);
         }
         case BLUE: {
            return createDiffuse (0f, 0f, 1f, 1f, 64f);
         }
         case GREEN: {
            return createDiffuse (0f, 1f, 0f, 1f, 64f);
         }
         case CYAN: {
            return createDiffuse (0f, 1f, 1f, 1f, 64f);
         }
         case MAGENTA: {
            return createDiffuse (1f, 0f, 1f, 1f, 64f);
         }
         case YELLOW: {
            return createDiffuse (1f, 1f, 0f, 1f, 64f);
         }
         case BLACK: {
            return createDiffuse (0f, 0f, 0f, 1f, 64f);
         }
         case GRAY: {
            return createDiffuse (0.5f, 0.5f, 0.5f, 1f, 32f);
         }
         case SILVER: {
            return createDiffuse (0.5f, 0.45f, 0.4f, 1f, 128f);
         }
         case GOLD: {
            return createDiffuse (0.93f, 0.8f, 0.063f, 1f, 128f);
         }
         default: {
            throw new ArrayIndexOutOfBoundsException (code);
         }
      }
   }

   /**
    * Returns true if this material is exactly equal to another material.
    * 
    * @param mat
    * material to compare to
    * @return true if the materials are equal
    */
   public boolean equal (Material mat) {
      for (int i = 0; i < 3; i++) {
         if (ambient[i] != mat.ambient[i]) {
            return false;
         }
         if (specular[i] != mat.specular[i]) {
            return false;
         }
         if (diffuse[i] != mat.diffuse[i]) {
            return false;
         }
         if (emission[i] != mat.emission[i]) {
            return false;
         }
      }
      if (shininess != mat.shininess) {
         return false;
      }
      if (diffuseAmbienceFactor != mat.diffuseAmbienceFactor) {
         return false;
      }
      // alpha value
      if (diffuse[3] != mat.diffuse[3]) {
         return false;
      }
      return true;
   }

   public void setAmbienceCoefficient(float ambience) {
      diffuseAmbienceFactor = ambience;
   }
   
   public float getAmbienceCoefficient() {
      return diffuseAmbienceFactor;
   }
}
