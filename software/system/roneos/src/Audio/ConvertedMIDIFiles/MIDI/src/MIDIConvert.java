
import java.io.*;
//import java.util.ArrayList;
class MIDIConvert{
	public static void main(String args[]) throws FileNotFoundException{
		GenerateFile file = new GenerateFile(args[0],args[1]);
		//GenerateFile file = new GenerateFile("Boogie_Woogie.txt","Boogie");
		file.generate();
		System.out.println("Done.");
	}
}