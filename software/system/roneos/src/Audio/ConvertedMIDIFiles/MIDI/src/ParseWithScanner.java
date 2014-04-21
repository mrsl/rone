import java.io.*;
import java.util.ArrayList;
import java.util.Scanner;
/*
 * @brief Class used to parse the BottomBoard file 
 * 
 * @since July 9, 2013
 * @author Lauren Schmidt
 */

public class ParseWithScanner{
 
	private final File fFile;
	private ArrayList<String> hexList;
	private int index;
	
	
	/*
	 * @brief Constructor for ParseWithScanner object
	 * 
	 * @param aFileName name of file containing BottomBoard info
	 */
	public ParseWithScanner(String aFileName){
		fFile = new File(aFileName);
		hexList = new ArrayList<String>();
		index = 0;
	}
	
	
	/*
	 * @brief Gets the list of parsed CAddresses
	 * @returns the cAddress list 
	 */
	public ArrayList<String> getAddresses(){
		return hexList;
	}
	
	
	/*
	 * @brief Processes the fFile
	 * 
	 * @returns void
	 */
	public final void processLineByLine() throws FileNotFoundException {
		Scanner scanner = new Scanner(new FileReader(fFile));
		try {
			//scan each line and process
			while (scanner.hasNextLine()){
				processLine(scanner.nextLine());
			}
		}
		finally {
			//end of file, stop scanning
			scanner.close();
			addSpace();
		}
	}//end processLineByLine()
  
	
	/*
	 * @brief Processes each line for addresses or hex values
	 * 
	 * @returns void
	 */
	private void processLine(String aLine){
		if(aLine.equals("")){
			//do nothing
		}
		else {
			//System.out.println("Line: " + aLine);
			String[] allAdd = aLine.split(" ");
			for (int i = 0;i<allAdd.length;i++){
				hexList.add("0x" + allAdd[i].substring(0,2));
				index ++;
				if(allAdd[i].length()>2){
					hexList.add("0x" + allAdd[i].substring(2));
					index ++;
				}
			}
		}
	}//end processLine()
	
	private void addSpace(){
		for (int i = 0; i< 128; i++){
			hexList.add("0x00");
			index ++;
		}
	}
	/*
	 * @brief Gets the number of addresses found in the fFile
	 * 
	 * @returns the number of addresses parsed
	 */
	public int getNumAdd(){
		return index;
	}
	
}//end ParseWithScanner class 