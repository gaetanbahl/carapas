/**
 * Copyright (c) 2015, by the Authors: Antonio Sanchez (UBC)
 *
 * This software is freely available under a 2-clause BSD license. Please see
 * the LICENSE file in the ArtiSynth distribution directory for details.
 */

package maspack.image.dicom;

import java.io.BufferedOutputStream;
import java.io.ByteArrayInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.util.List;

import artisynth.core.util.ArtisynthPath;
import maspack.util.BinaryFileInputStream;
import maspack.util.BitInputStream;

/**
 * Relies on the ImageMagick command-line utilities to decode an image slice 
 * @author Antonio
 *
 */
public class DicomImageDecoderGDCM implements DicomImageDecoder {
 
   /**
    * Creates a GDCM decoder
    */
   public DicomImageDecoderGDCM() {
      
   }

   public boolean isValid() {
      return true;
   }


   public boolean canDecode(DicomHeader header) {

      DicomTransferSyntax dts = header.getTransferSyntax();

      if (dts.uid.equals("1.2.840.10008.1.2.4.90")
         || dts.uid.equals("1.2.840.10008.1.2.4.91")
         || dts.uid.equals("1.2.840.10008.1.2.4.92")
         || dts.uid.equals("1.2.840.10008.1.2.4.93")
         || dts.uid.equals("1.2.840.10008.1.2.4.70")) {
         return false;
      } else {
         System.out.println("GDCM: unknown transfer syntax '" + dts.uid + 
            "', attempting to decode anyways...");
      }
      
      return false;
   }

   @Override
   public boolean decodeFrames(
      DicomHeader header, BinaryFileInputStream bin,
      List<DicomPixelBuffer> frames)  throws IOException {
      
      // ready in case of reset
      bin.mark(bin.available());
      
      
      
      
      return false;
   }

}
