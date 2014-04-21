import java.io.*;
/*
 * @brief Class used to create c file
 * 
 * @since July 9, 2013
 * @author Lauren Schmidt
 */
class GenerateFile{
	private String name;
	private String load;
	private LoadPath path;
	
	public GenerateFile(String loadFile, String fileName) throws FileNotFoundException{
		name = fileName;
		load = loadFile;
		path = new LoadPath("loadPath.txt");
	}
	
	public void generate() throws FileNotFoundException{
		try {
				File cfile = new File(path.getPath() + "mrone/robotcode/roneLibs/roneos/src/Audio/ConvertedMIDIFiles/MIDIFile_" + name + ".c");
				File hfile = new File(path.getPath() + "mrone/robotcode/roneLibs/roneos/src/Audio/ConvertedMIDIFiles/MIDIFile_" + name + ".h");
				if(!cfile.exists()){
					cfile.createNewFile();
				}
				if(!hfile.exists()){
					hfile.createNewFile();
				}

				//generate new header file
				BuildCFile makeCFile = new BuildCFile(load,name);
				BuildHFile makeHFile = new BuildHFile(name);
				FileWriter fwc = new FileWriter(cfile.getAbsoluteFile());
				FileWriter fwh = new FileWriter(hfile.getAbsoluteFile());
				BufferedWriter bwc = new BufferedWriter(fwc);
				BufferedWriter bwh = new BufferedWriter(fwh);
				bwc.write(makeCFile.getCFile());
				bwh.write(makeHFile.getHFile());
				bwc.close();
				bwh.close();
				
		}
		catch (IOException e){
			e.printStackTrace();
		}
	}
}