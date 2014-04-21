import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.Date;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
/*
 * @brief Used to build the text of the new c file using parsed info from the necessary documents
 * 
 * @since July 9, 2013
 * @author Lauren Schmidt
 */
public class BuildCFile{
	private String cFile;
	private String file;
	private String name;
	private ParseWithScanner parser;
	
	
	/*
	 * @brief constructor for a BuildCFile
	 * 
	 * Note: The cFile is automatically build upon initialization using the input information
	 * 
	 * @param boardFilename name of the bottom board file to parse
	 * @param versionFile name of the file containing the version number of the code
	 * @param hardwareNum the digit (1 or 2) indicating if the hardware if V11 or V12
	 */
	public BuildCFile(String filename, String structname) throws FileNotFoundException{
		file = filename;
		name = structname;
		parser = new ParseWithScanner(file);
		parser.processLineByLine();
		cFile = "";
		buildFile();
		
	}
	
	
	/*
	 * @brief Generates the text for the c file
	 * 
	 * @returns void
	 */
	public void buildFile(){
		buildHeader();
		buildSectionStructure();
		buildSectionDataArrays();
		buildEnd();
	}
	
	
	/*
	 * @brief Generates the header part of the file; includes time and date of generation
	 * 
	 * @returns void
	 */
	private void buildHeader(){
		DateFormat dateFormat = new SimpleDateFormat("MM/dd/yyyy HH:mm");
		//get current date time with Date()
		Date date = new Date();
		cFile = "/*\n*  Auto-Generated On: ";
		cFile += dateFormat.format(date);
		cFile += "\n*  Author: Lauren Schmidt\n*/";
		cFile += "\n\n#if (defined(RONE_V9) || defined(RONE_V12))";
		cFile += "\n#include \"roneos.h\"";
	}
	
	
	
	/*
	 * @brief Generates the section structure
	 * 
	 * @returns void
	 */
	private void buildSectionStructure(){
		cFile += "\n\nconst char MIDIFile_" + name +"[] = {\n	";
	}
	
	
	/*
	 * @brief Generates the Data Arrays using the boardFilename from the constructor
	 * 
	 * @returns void
	 */
	private void buildSectionDataArrays(){
		ArrayList<String> hexList = parser.getAddresses();
		int i = 1;
		while(i <= hexList.size()){
			cFile += hexList.get(i-1);
			if (i != hexList.size()){ 
				cFile += ", ";
			}else{
				cFile += " };";
			}
			if(i%10 == 0){
				cFile += "\n	";
			}
			i++;
		}
	}
	
	
	private void buildEnd(){
		cFile += "\n\n#endif";
	}
	
	
	
	/*
	 * @brief Gets the String corresponding to the CFile text
	 * 
	 * @returns cFile, the text of the converted cFile
	 */
	public String getCFile(){
		return cFile;
	}
	
	
	
	/*
	 * @brief Gets the String corresponding to the file name
	 * 
	 * @returns the string corresponding to the file name
	 */
	public String getFile(){
		return file;
	}
	
	
	/*
	 * @brief Prints the String cFile to the console
	 * Note: Used for debugging
	 * 
	 * @returns void
	 */
	public void printCFile(){
		System.out.println(cFile);
	}
}