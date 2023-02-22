
#ifndef _HARDWARE_H_
#define _HARDWARE_H_

//
//	hardware.h
//
//	-------------------------------------------------------------------
//! @addtogroup HARDWARE
//! <h3> API Functions for hardware access </h3>
//!
//! These functions can be used to access hardware resources. Most
//! resources are simple jobs like LED control. The functions are
//! made in a blocking style returning when the work is done.
//! Please note that not all resources are available on all targets.
//!
//! <h3> Targets: </h3>
//! - PCAN-Router @n
//! - PCAN-Router Pro @n
//! - PCAN-RS-232
//!
//	-------------------------------------------------------------------
//
//	Copyright (C) 1999-2011  PEAK-System Technik GmbH, Darmstadt
//	more Info at http://www.peak-system.com 
//
//! @{


////////////////////////////////////////////////////////////
//! @name errors
//! A function returns one of these errors:
//! @{
#define	HW_ERR_OK				0		//!< OK, no error
#define	HW_ERR_ILLPARAMVAL	1		//!< invalid parameter value
#define	HW_ERR_RESOURCE		2		//!< hardware resource is not available
/*! @}*/


////////////////////////////////////////////////////////////
//! @name beeper-tones
//! The beeper can be set ON or OFF only
//! @{
#define	HW_BEEPER_OFF		0			//!< beeper OFF
#define	HW_BEEPER_ON		0xFF		//!< beeper ON
/*! @}*/


////////////////////////////////////////////////////////////
//! @name LED-handles
//! Handles to the LEDs. Not all LEDs are available on all targets.
//! @{
#define	HW_LED_STATUS		0		//!< status LED
#define	HW_LED_CAN1			1		//!< LED CAN1
#define	HW_LED_CAN2			2		//!< LED CAN2
#define	HW_LED_CAN3			3		//!< LED CAN3
#define	HW_LED_CAN4			4		//!< LED CAN4
#define	HW_LED_CAN1b		5		//!< second LED CAN1
#define	HW_LED_CAN2b		6		//!< second LED CAN2
#define	HW_LED_CAN3b		7		//!< second LED CAN3
#define	HW_LED_CAN4b		8		//!< second LED CAN4
/*! @}*/


////////////////////////////////////////////////////////////
//! @name LED-colors
//! Colors used for the LEDs
//! @{
#define	HW_LED_OFF			0		//!< LED switch OFF
#define	HW_LED_RED			1		//!< LED switch red
#define	HW_LED_GREEN		2		//!< LED switch green
#define	HW_LED_ORANGE		3		//!< LED switch orange
/*! @}*/


////////////////////////////////////////////////////////////
//! @name types
//! Basetypes for hardware functions
//! @{
#define	HWStatus_t		u32_t			//!< status code type for API functions
#define	LEDHandle_t		u8_t			//!< handle type
#define	LEDColor_t		u8_t			//!< color type
/*! @}*/


#ifdef __cplusplus
extern "C" {
#endif


////////////////////////////////////////////////////////////
//! @name functions
//! This section will describe the API functions. The functions will target
//! to the main() level.
//! @{


//! @brief
//! Initialize hardware (port pins, cpu internals etc.). Call this on
//! main() entry.
//!
//! @return		one error of HW_ERR_...
HWStatus_t		HW_Init ( void);



//! @brief
//! Retrieve the module ID from the solder pins or coding switch.
//!
//! @param		buffer		Buffer for the ID
//!
//! @return		one error of HW_ERR_...
HWStatus_t		HW_GetModuleID (	u8_t  *buffer);


//! @brief
//! Retrieve the hex-switches from hardware. Only available on
//! PCAN-Router-DR
//!
//! @param		buffer		Buffer for the values, use buff[3]
//!
//! @return		one error of HW_ERR_...
HWStatus_t  HW_GetHexSwitch ( u8_t  *buffer);


//! @brief
//! Read digital inputs. Each bit will represent a digital pin.
//! Not available on PCAN-Router Pro
//!
//! @param		buffer		Buffer for DIN-value
//!
//! @return		one error of HW_ERR_...
HWStatus_t  HW_GetDIN ( u32_t  *buffer);


//! @brief
//! Set digital outputs. Each bit will represent a digital pin.
//! Only available on PCAN-RS-232.
//! 
//! @param		buffer		Buffer for DOUT-value
//!
//! @return		one error of HW_ERR_...
HWStatus_t  HW_SetDOUT ( u32_t  *buffer);



//! @brief
//! Set a LED to a specific color.
//!
//! @param		hLED		handle of the LED, see LED-handles
//! @param		color		color to set, see HW_LED_RED ...
//!
//! @return		one error of HW_ERR_...
HWStatus_t		HW_SetLED (	LEDHandle_t  hLED, LEDColor_t  color);



//! @brief
//! Jumps to the Bootloader. This function will never return. Using '0' for
//! the baudrate will instruct the Bootloader to use the default baudrate.
//!
//! @param		baudrate		baudrate for the bootloader, see can_user.h
//!
//! @return		this function will never return
void		HW_JumpToBootloader ( u32_t  baudrate);



//! @brief
//! Set beeper ON or OFF. This function is only available on PCAN-Router Pro.
//!
//! @param		tone		tone for the beeper, see HW_BEEPER_...
//!
//! @return		one error of HW_ERR_...
HWStatus_t		HW_SetBeeper ( u8_t  tone);



//! @brief
//! Switch OFF the hardware. This function will never return.
//! Only available on PCAN-Router Pro.
void		HW_SwitchOFF ( void);

/*! @}*/

#ifdef __cplusplus
}
#endif

/*! @}*/

#endif

