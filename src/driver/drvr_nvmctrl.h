/*
 * drvr_nvmctrl.h
 *
 * Created: 2/14/2018 10:42:56 AM
 *  Author: akudlacek
 */ 


#ifndef DRVR_NVMCTRL_H_
#define DRVR_NVMCTRL_H_


#include <stdint.h>
#include "sam.h"


/**************************************************************************************************
*                                             DEFINES
*************************************************^************************************************/
/*NVM User Row Mapping for samc21*/
typedef union nvm_usr_row_t
{
	struct
	{
		uint64_t           BOOTPROT:3;   //Used to select one of eight different bootloader sizes.
		uint64_t                   :1;   //Reserved
		uint64_t             EEPROM:3;   //Used to select one of eight different EEPROM sizes.
		uint64_t                   :1;   //Reserved
		uint64_t       BODVDD_LEVEL:6;   //BODVDD Threshold Level at power on.
		uint64_t     BODVDD_DISABLE:1;   //BODVDD Disable at power on.
		uint64_t      BODVDD_ACTION:2;   //BODVDD Action at power on.
		uint64_t                   :9;   //Reserved - Voltage Regulator Internal BOD (BODCORE) configuration. These bits are written in production and must not be changed.
		uint64_t         WDT_ENABLE:1;   //WDT Enable at power on.
		uint64_t      WDT_ALWAYS_ON:1;   //WDT Always-On at power on.
		uint64_t         WDT_PERIOD:4;   //WDT Period at power on.
		uint64_t         WDT_WINDOW:4;   //WDT Window mode time-out at power on.
		uint64_t       WDT_EWOFFSET:4;   //WDT Early Warning Interrupt Time Offset at power on.
		uint64_t            WDT_WEN:1;   //WDT Timer Window Mode Enable at power on.
		uint64_t  BODVDD_HYSTERESIS:1;   //BODVDD Hysteresis configuration at power on.
		uint64_t                   :1;   //Reserved - Voltage Regulator Internal BOD (BODCORE) configuration. These bits are written in production and must not be changed.
		uint64_t                   :5;   //Reserved
		uint64_t              LOCK:16;   //NVM Region Lock Bits.
	} bit;                               //Structure used for bit  access
	uint64_t reg;                        //used for register access
} nvm_usr_row_t;

/*NVM Software Calibration Area Mapping for samc21*/
typedef union nvm_sw_calib_t
{
	struct
	{
		uint64_t     ADC0_LINEARITY:3;   //ADC0 Linearity Calibration. Should be written to the CALIB register. (ADC0_FUSES_BIASREFBUF)
		uint64_t       ADC0_BIASCAL:3;   //ADC0 Bias Calibration. Should be written to the CALIB register. (ADC0_FUSES_BIASCOMP)
		uint64_t     ADC1_LINEARITY:3;   //ADC1 Linearity Calibration. Should be written to the CALIB register. (ADC1_FUSES_BIASREFBUF)
		uint64_t       ADC1_BIASCAL:3;   //ADC1 Bias Calibration. Should be written to the CALIB register. (ADC1_FUSES_BIASCOMP)
		uint64_t         OSC32K_CAL:7;   //OSC32K Calibration. Should be written to OSC32K register.
		uint64_t         CAL48M_5V:22;   //OSC48M Calibration: VDD range 3.6V to 5.5V. Should be written to the CAL48M register.
		uint64_t        CAL48M_3V3:22;   //OSC48M Calibration: VDD range 2.7V to 3.6V. Should be written to the CAL48M register.
		uint64_t                   :1;   //Reserved
	} bit;                               //Structure used for bit  access
	uint64_t reg;                        //used for register access
} nvm_sw_calib_t;

/*NVM Temperature Calibration Area Mapping for samc21*/
typedef union nvm_temp_calib_t
{
	struct
	{
		uint64_t         TSENS_TCAL:6;   //TSENS Temperature Calibration. Should be written to the TSENS CAL register.
		uint64_t         TSENS_FCAL:6;   //TSENS Frequency Calibration. Should be written to the TSENS CAL register.
		uint64_t        TSENS_GAIN:24;   //TSENS Gain Calibration. Should be written to the TSENS GAIN register.
		uint64_t      TSENS_OFFSET:24;   //TSENS Offset Calibration. Should be written to TSENS OFFSET register.
		uint64_t                   :4;   //Reserved
	} bit;                               //Structure used for bit  access
	uint64_t reg;                        //used for register access
} nvm_temp_calib_t;

/*NVM structures - Set as read only, the sections that are modifiable must use nvmctrl functions*/
#define NVM_USR_ROW    ((const volatile nvm_usr_row_t    * const)0x00804000UL)   //NVM User Row Mapping
#define NVM_SW_CALIB   ((const volatile nvm_sw_calib_t   * const)0x00806020UL)   //NVM Software Calibration Area Mapping
#define NVM_TEMP_CALIB ((const volatile nvm_temp_calib_t * const)0x00806030UL)   //NVM Temperature Calibration Area Mapping

/*Serial Number for samc21 - Read only*/
#define SAMC21_SN_WORD0 *((const volatile uint32_t * const)0x0080A00CUL) //WORD 0
#define SAMC21_SN_WORD1 *((const volatile uint32_t * const)0x0080A040UL) //WORD 1
#define SAMC21_SN_WORD2 *((const volatile uint32_t * const)0x0080A044UL) //WORD 2
#define SAMC21_SN_WORD3 *((const volatile uint32_t * const)0x0080A048UL) //WORD 3

/*NVM controller return codes*/
typedef enum drvr_nvmctrl_return_t
{
	NVM_SUCCESS,
	NVM_ABORTED,
	NVM_BUSY,
	NVM_ERR_IO,
	NVM_ERR_INVALID_ARG,
	NVM_ERR_BAD_ADDRESS
} drvr_nvmctrl_return_t;

/*NVM controller commands*/
typedef enum nvm_command_t
{
	NVM_COMMAND_ERASE_ROW                  = NVMCTRL_CTRLA_CMD_ER,      //Erases the addressed memory row
	NVM_COMMAND_WRITE_PAGE                 = NVMCTRL_CTRLA_CMD_WP,      //Write the contents of the page buffer to the addressed memory page
	NVM_COMMAND_ERASE_AUX_ROW              = NVMCTRL_CTRLA_CMD_EAR,     //Erases the addressed auxiliary memory row. NOTE: This command can only be given when the security bit is not set.
	NVM_COMMAND_WRITE_AUX_ROW              = NVMCTRL_CTRLA_CMD_WAP,     //Write the contents of the page buffer to the addressed auxiliary memory row. NOTE: This command can only be given when the security bit is not set.
	NVM_COMMAND_LOCK_REGION                = NVMCTRL_CTRLA_CMD_LR,      //Locks the addressed memory region, preventing further modifications until the region is unlocked or the device is erased
	NVM_COMMAND_UNLOCK_REGION              = NVMCTRL_CTRLA_CMD_UR,      //Unlocks the addressed memory region, allowing the region contents to be modified
	NVM_COMMAND_PAGE_BUFFER_CLEAR          = NVMCTRL_CTRLA_CMD_PBC,     //Clears the page buffer of the NVM controller, resetting the contents to all zero values
	NVM_COMMAND_SET_SECURITY_BIT           = NVMCTRL_CTRLA_CMD_SSB,     //Sets the device security bit, disallowing the changing of lock bits and auxiliary row data until a chip erase has been performed
	NVM_COMMAND_ENTER_LOW_POWER_MODE       = NVMCTRL_CTRLA_CMD_SPRM,    //Enter power reduction mode in the NVM controller to reduce the power consumption of the system
	NVM_COMMAND_EXIT_LOW_POWER_MODE        = NVMCTRL_CTRLA_CMD_CPRM,    //Exit power reduction mode in the NVM controller to allow other NVM commands to be issued
	NVM_COMMAND_RWWEE_ERASE_ROW            = NVMCTRL_CTRLA_CMD_RWWEEER, //Read while write (RWW) EEPROM area erase row
	NVM_COMMAND_RWWEE_WRITE_PAGE           = NVMCTRL_CTRLA_CMD_RWWEEWP  //RWW EEPROM write page
} nvm_command_t;

/*NVM memory parameter structure.*/
/*Structure containing the memory layout parameters of the NVM module.*/
typedef struct nvm_parameters_t
{
	uint8_t  page_size;                   //Number of bytes per page
	uint16_t nvm_number_of_pages;         //Number of pages in the main array
	uint32_t eeprom_number_of_pages;      //Size of the emulated EEPROM memory section configured in the NVM auxiliary memory space
	uint32_t bootloader_number_of_pages;  //Size of the Bootloader memory section configured in the NVM auxiliary memory space
	uint16_t rww_eeprom_number_of_pages;  //Number of pages in read while write EEPROM (RWWEE) emulation area
} nvm_parameters_t;


/**************************************************************************************************
*                                            PROTOTYPES
*************************************************^************************************************/
//todo: type correct
void                  drvr_nvmctrl_init          (const uint16_t cpu_voltage_mv, const uint32_t cpu_freq_hz);
drvr_nvmctrl_return_t drvr_nvmctrl_exe_cmd       (const nvm_command_t command, const uint32_t address); 
drvr_nvmctrl_return_t drvr_nvmctrl_write_buffer  (const uint32_t destination_address, const uint8_t * const buffer, const uint16_t length);
drvr_nvmctrl_return_t drvr_nvmctrl_read_buffer   (const uint32_t source_address, uint8_t * const buffer, const uint16_t length);
drvr_nvmctrl_return_t drvr_nvmctrl_erase_row     (const uint32_t row_address);
void                  drvr_nvmctrl_get_parameters(nvm_parameters_t * const parameters);
uint8_t               drvr_nvmctrl_is_page_locked(const uint16_t page_number);
uint8_t               nvm_is_ready               (void);


#endif /* DRVR_NVMCTRL_H_ */