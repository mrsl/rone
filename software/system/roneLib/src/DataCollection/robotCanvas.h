/**
 * @file robotCanvas.h
 * @brief Sends radio commands so that the robots can be plotted on a remote computer.
 * @details
 * @since April 23, 2011
 * @author James McLurkin
 */

#ifndef ROBOTCANVAS_H_
#define ROBOTCANVAS_H_

/**
 * @brief Prints rc,newPage
 * @returns void
 */
void rcNewPage(void);


//TODO Should this be deleted, it is commented out
void rcUpdate(void);


/**
 * @brief Prints rc,
 * @returns void
 */
void rcUpdatePage(void);


/**
 * @brief Prints rc,'command'
 * @param command that wants to be printed
 * @returns void
 */
void rcCommand(char* command);


/**
 * @brief Prints rc,setLayer,'layer'
 * @param layer that wants to be printed
 * @returns void
 */
void rcSetLayer(uint8 layer);


/**
 * @brief Prints rc,val,'name','val'
 * @param name that wants to be printed
 * @param val that wants to be printed
 * @returns void
 */
void rcValue(char* name, uint32 val);


/**
 * @brief Prints data on neighbor and all its neihbors
 * @param nbr First neighbor to be printed
 * @param round is printed with first neighbors
 * @returns void
 */
void rcNbrPrint(Nbr* nbr, uint8 round);


/**
 * @brief Prints name, id and position
 * @param name that wants to be printed
 * @param id that wants to be printed
 * @param posePtr that wants to be printed
 * @returns void
 */
void rcPosePrint(char* name, uint8 id, Pose* posePtr);


/**
 * @brief Prints all neighbors and all their neighbors
 * @returns void
 */
void rcNbrPrintAll(void);


/**
 * @brief Prints the id and round of all neibors
 * @returns void
 */
void rcNbrVarsPrintAll(void);


/**
 * @brief Prints message and size of all messages
 * @returns void
 */
void rcNbrVarsPrintHeader(void);

#endif /* ROBOTCANVAS_H_ */
