/*
 * 
 */

public class BuildHFile{
	private String filename;
	private String hFile;
	public BuildHFile(String name){
		filename = name;
		hFile = "";
		build();
	}
	
	private void build(){
		hFile += "\n#ifndef " + filename;
		hFile += "\n#define " + filename;
		hFile += "\n\n#include \"roneos.h\"";
		hFile += "\n\nextern const char MIDIFile_" + filename + "[];";
		hFile += "\n\n#endif";
	}
	
	public String getHFile(){
		return hFile;
	}
}
