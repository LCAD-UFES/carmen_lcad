package LDLoad;

import java.util.*;
import java.util.zip.*;
import java.io.*;

/* The idea and implementation of this came from Ed Olson and the MasLab
   codebase. */

public class LDLoad {

	static public boolean loadFromJar(String jarFilename, String soFilename)
	{    
		String items = jarFilename;
		{
			if (new File(items).exists()) 
			{
				try{
					int size = 0;
					boolean found = false;

					ZipFile zf=new ZipFile(items);
					Enumeration e=zf.entries();
					while (e.hasMoreElements()) {
						ZipEntry ze=(ZipEntry)e.nextElement();
						System.err.println(ze.getName());
						if (ze.getName().equals(soFilename)) {
							size = (int)ze.getSize();
							found = true;
							break;
						}
					}
					zf.close();

					if (!found)
						throw new RuntimeException(jarFilename+" does not contain .so "+soFilename);

					FileInputStream fis = new FileInputStream(items);
					BufferedInputStream bis = new BufferedInputStream(fis);
					ZipInputStream zis = new ZipInputStream(bis);

					File soFile = new File(soFilename);

					File output = File.createTempFile("lib", soFile.getName());
					FileOutputStream fos = new FileOutputStream(output);
					ZipEntry ze = null;
					while ((ze = zis.getNextEntry()) != null) {
						System.err.println(ze.getName());
						if (ze.getName().equals(soFilename)) {
							int count = 0;
							byte[] result = new byte[size];
							while (count < size)
								count += zis.read(result,count,size-count);
							fos.write(result);
							break;
						}
					}
					fos.close();

					System.load(output.toString());
					output.delete();
					return true;
				}
				catch(IOException ioe){
					System.err.println(ioe);
				}
			}
		}
		throw new RuntimeException("Unable to load native library "+soFilename+
				" from "+jarFilename);
	}
}
