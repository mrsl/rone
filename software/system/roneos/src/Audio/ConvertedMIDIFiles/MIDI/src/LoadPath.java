import java.io.*;
/*
 * @brief Used to determine the user's distinct path to the mrone folder
 * 
 * @since July 9, 2013
 * @author Lauren Schmidt
 */
class LoadPath{
	private File load;
	private String path;
	
	/*
	 * @brief Constructs a LoadPath object
	 * 
	 * @param filename the name of the dummy file to create to parse the path
	 */
	public LoadPath(String filename) throws FileNotFoundException{
		try {
				load = new File(filename);

				// if file doesn't exists, then create it
				if (!load.exists()) {
					load.createNewFile();
				}
		
				//get the user's path to mrone contents
				path = load.getAbsolutePath();
				path = path.substring(0,path.indexOf("mrone"));
			} catch (IOException e) {
				e.printStackTrace();
		}
	}
	
	
	/*
	 * @brief Gets the computer's path to the mrone folder
	 * 
	 * @returns the String corresponding to the computer's path to the mrone folder
	 */
	public String getPath(){
		return path;
	}
}