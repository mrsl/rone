/**
 * @file charger.h
 * @brief initializes/enables/disables battery charger
 * @since Mar 26, 2011
 * @author James McLurking
 */

#ifndef CHARGER_H_
#define CHARGER_H_


/**
 * @brief Initializes charger.
 * does nothing for v12
 * Enables peripheral and sets the charger pin as output.
 * @returns void
 */
void charger_init(void);


/**
 * @brief Enables charger.
 * does nothing for v12
 *
 * @returns void
 */
void charger_enable(void);


/*
 * @brief Disables the charger.
 * does nothing for v12
 * @returns void
 */
void charger_disable(void);


/*
 * @brief Gets the status of the charger.
 * always true for v12
 * @returns true if the charger is enabled, false otherwise
 */
uint32 charger_get_status(void); // python rone

#endif /* CHARGER_H_ */
