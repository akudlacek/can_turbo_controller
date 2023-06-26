/*
 * drvr_nvmctrl.c
 *
 * Created: 2/14/2018 10:42:34 AM
 *  Author: akudlacek
 */ 


#include "drvr_nvmctrl.h"


/**************************************************************************************************
*                                             DEFINES
*************************************************^************************************************/
#define NVM_ERRORS_MASK   (NVMCTRL_STATUS_PROGE | NVMCTRL_STATUS_LOCKE | NVMCTRL_STATUS_NVME) //Mask for the error flags in the status register
#define NVM_MEMORY        ((volatile uint16_t * const)FLASH_ADDR)                             //Pointer to the NVM MEMORY region start address
#define NVM_USER_MEMORY   ((volatile uint16_t * const)NVMCTRL_USER)                           //Pointer to the NVM USER MEMORY region start address

/*Internal device instance struct*/
typedef struct
{
	uint16_t page_size;         //Number of bytes contained per page.
	uint16_t number_of_pages;   //Total number of pages in the NVM memory.
	uint8_t manual_page_write;  //If \c 0, a page write command will be issued automatically when the *  page buffer is full.
} _nvm_module_t;

/*NVM error flags - Possible NVM controller error codes, which can be returned by the NVM controller after a command is issued.*/
typedef enum
{
	NVM_ERROR_NONE = 0,                                            //No errors
	NVM_ERROR_LOCK = NVMCTRL_STATUS_NVME | NVMCTRL_STATUS_LOCKE,   //Lock error, a locked region was attempted accessed
	NVM_ERROR_PROG = NVMCTRL_STATUS_NVME | NVMCTRL_STATUS_PROGE,   //Program error, invalid command was executed
} nvm_error_t;


/**************************************************************************************************
*                                            VARIABLES
*************************************************^************************************************/
static _nvm_module_t _nvm_dev; //Instance of the internal device struct


/**************************************************************************************************
*                                            FUNCTIONS
*************************************************^************************************************/
/******************************************************************************
*  \brief NVMCTRL init
*
*  \note Upon power-up completion, the NVM Controller is operational without
*        any need for user configuration.
******************************************************************************/
void drvr_nvmctrl_init(const uint16_t cpu_voltage_mv, const uint32_t cpu_freq_hz)
{
	uint8_t num_wait_states = 0;
	
	if(cpu_voltage_mv > 4500)
	{
		//20MHz
		if(cpu_freq_hz <= 20000000)
		{
			num_wait_states = 0;
		}
		//38MHz
		else if(cpu_freq_hz <= 38000000)
		{
			num_wait_states = 1;
		}
		//48MHz
		else if(cpu_freq_hz <= 48000000)
		{
			num_wait_states = 2;
		}
		else
		{
			num_wait_states = 3;
		}
	}
	else //cpu_voltage_mv <= 4500
	{
		//19MHz
		if(cpu_freq_hz <= 19000000)
		{
			num_wait_states = 0;
		}
		//38MHz
		else if(cpu_freq_hz <= 38000000)
		{
			num_wait_states = 1;
		}
		//48MHz
		else if(cpu_freq_hz <= 48000000)
		{
			num_wait_states = 2;
		}
		else
		{
			num_wait_states = 3;
		}
	}
	
	NVMCTRL->CTRLB.bit.RWS      = num_wait_states; //NVM Read Wait States
	
	//Cache - there is an errata for this. Looks like RWW EEPROM Cache has some problems.
	//Also the old atmel datasheet and the new microchip one have differences on this section
	NVMCTRL->CTRLB.bit.CACHEDIS =             0x0; //RWW EEPROM Cache = disabled & NVM Cache = enabled;
	NVMCTRL->CTRLB.bit.READMODE =             0x0; //NO_MISS_PENALTY - best performance
	NVMCTRL->CTRLB.bit.SLEEPPRM =             0x0; //WAKEUPACCESS
	NVMCTRL->CTRLB.bit.MANW     =               0; //Manual page write disabled(Automatic)
	
	/* Initialize the internal device struct */
	_nvm_dev.page_size         = (8 << NVMCTRL->PARAM.bit.PSZ);
	_nvm_dev.number_of_pages   =       NVMCTRL->PARAM.bit.NVMP;
	_nvm_dev.manual_page_write =       NVMCTRL->CTRLB.bit.MANW;
}

/******************************************************************************
 * \brief Executes a command on the NVM controller.
 *
 * Executes an asynchronous command on the NVM controller, to perform a requested
 * action such as an NVM page read or write operation.
 *
 * \note The function will return before the execution of the given command is
 *       completed.
 *
 * \param[in] command    Command to issue to the NVM controller
 * \param[in] address    Address to pass to the NVM controller in NVM memory
 *                       space
 * \param[in] parameter  Parameter to pass to the NVM controller, not used
 *                       for this driver
 *
 * \return Status of the attempt to execute a command.
 *
 * \retval NVM_SUCCESS               If the command was accepted and execution
 *                                 is now in progress
 * \retval NVM_BUSY             If the NVM controller was already busy
 *                                 executing a command when the new command
 *                                 was issued
 * \retval NVM_ERR_IO           If the command was invalid due to memory or
 *                                 security locking
 * \retval NVM_ERR_INVALID_ARG  If the given command was invalid or
 *                                 unsupported
 * \retval NVM_ERR_BAD_ADDRESS  If the given address was invalid
******************************************************************************/
drvr_nvmctrl_return_t drvr_nvmctrl_exe_cmd(const nvm_command_t command, const uint32_t address)
{
	uint32_t ctrlb_bak;

	/* Check that the address given is valid  */
	if (address > ((uint32_t)_nvm_dev.page_size * _nvm_dev.number_of_pages)	&& !(address >= NVMCTRL_AUX0_ADDRESS && address <= NVMCTRL_AUX1_ADDRESS ))
	{
		if(address >= ((uint32_t)NVMCTRL_RWW_EEPROM_SIZE + NVMCTRL_RWW_EEPROM_ADDR) || address < NVMCTRL_RWW_EEPROM_ADDR)
		{
			return NVM_ERR_BAD_ADDRESS;
		}
	}

	/* Get a pointer to the module hardware instance */
	Nvmctrl *const nvm_module = NVMCTRL;

	/* Turn off cache before issuing flash commands */
	ctrlb_bak = nvm_module->CTRLB.reg;

	nvm_module->CTRLB.reg = ((ctrlb_bak &(~(NVMCTRL_CTRLB_CACHEDIS(0x2)))) | NVMCTRL_CTRLB_CACHEDIS(0x1));

	/* Clear error flags */
	nvm_module->STATUS.reg = NVMCTRL_STATUS_MASK;

	/* Check if the module is busy */
	if (!nvm_is_ready()) {
		/* Restore the setting */
		nvm_module->CTRLB.reg = ctrlb_bak;
		return NVM_BUSY;
	}

	switch (command) {

		/* Commands requiring address (protected) */
		case NVM_COMMAND_ERASE_AUX_ROW:
		case NVM_COMMAND_WRITE_AUX_ROW:

			/* Auxiliary space cannot be accessed if the security bit is set */
			if (nvm_module->STATUS.reg & NVMCTRL_STATUS_SB) {
				/* Restore the setting */
				nvm_module->CTRLB.reg = ctrlb_bak;
				return NVM_ERR_IO;
			}

			/* Set address, command will be issued elsewhere */
			nvm_module->ADDR.reg = (uintptr_t)&NVM_MEMORY[address / 4];
			break;

		/* Commands requiring address (unprotected) */
		case NVM_COMMAND_ERASE_ROW:
		case NVM_COMMAND_WRITE_PAGE:
		case NVM_COMMAND_LOCK_REGION:
		case NVM_COMMAND_UNLOCK_REGION:
		case NVM_COMMAND_RWWEE_ERASE_ROW:
		case NVM_COMMAND_RWWEE_WRITE_PAGE:

			/* Set address, command will be issued elsewhere */
			nvm_module->ADDR.reg = (uintptr_t)&NVM_MEMORY[address / 4];
			break;

		/* Commands not requiring address */
		case NVM_COMMAND_PAGE_BUFFER_CLEAR:
		case NVM_COMMAND_SET_SECURITY_BIT:
		case NVM_COMMAND_ENTER_LOW_POWER_MODE:
		case NVM_COMMAND_EXIT_LOW_POWER_MODE:
			break;

		default:
			/* Restore the setting */
			nvm_module->CTRLB.reg = ctrlb_bak;
			return NVM_ERR_INVALID_ARG;
	}

	/* Set command */
	nvm_module->CTRLA.reg = command | NVMCTRL_CTRLA_CMDEX_KEY;

	/* Wait for the NVM controller to become ready */
	while (!nvm_is_ready()) {
	}

	/* Restore the setting */
	nvm_module->CTRLB.reg = ctrlb_bak;

	return NVM_SUCCESS;
}

/******************************************************************************
 * \brief Writes a number of bytes to a page in the NVM memory region.
 *
 * Writes from a buffer to a given page address in the NVM memory.
 *
 * \param[in]  destination_address  Destination page address to write to
 * \param[in]  buffer               Pointer to buffer where the data to write is
 *                                  stored
 * \param[in]  length               Number of bytes in the page to write
 *
 * \note If writing to a page that has previously been written to, the page's
 *       row should be erased (via \ref nvm_erase_row()) before attempting to
 *       write new data to the page.
 *
 * \note For SAM D21 RWW devices, see \c SAMD21_64K, command \c NVM_COMMAND_RWWEE_WRITE_PAGE
 * must be executed before any other commands after writing a page,
 * refer to errata 13588.
 *
 * \note If manual write mode is enabled, the write command must be executed after
 * this function, otherwise the data will not write to NVM from page buffer.
 *
 * \return Status of the attempt to write a page.
 *
 * \retval NVM_SUCCESS               Requested NVM memory page was successfully
 *                                 read
 * \retval NVM_BUSY             NVM controller was busy when the operation
 *                                 was attempted
 * \retval NVM_ERR_BAD_ADDRESS  The requested address was outside the
 *                                 acceptable range of the NVM memory region or
 *                                 not aligned to the start of a page
 * \retval NVM_ERR_INVALID_ARG  The supplied write length was invalid
******************************************************************************/
drvr_nvmctrl_return_t drvr_nvmctrl_write_buffer(const uint32_t destination_address, const uint8_t * const buffer, const uint16_t length)
{
	uint8_t is_rww_eeprom = 0;

	/* Check if the destination address is valid */
	if(destination_address > ((uint32_t)_nvm_dev.page_size * _nvm_dev.number_of_pages))
	{
		if(destination_address >= ((uint32_t)NVMCTRL_RWW_EEPROM_SIZE + NVMCTRL_RWW_EEPROM_ADDR) || destination_address < NVMCTRL_RWW_EEPROM_ADDR)
		{
			return NVM_ERR_BAD_ADDRESS;
		}
		is_rww_eeprom = 1;
	}

	/* Check if the write address not aligned to the start of a page */
	if(destination_address & (_nvm_dev.page_size - 1))
	{
		return NVM_ERR_BAD_ADDRESS;
	}

	/* Check if the write length is longer than an NVM page */
	if(length > _nvm_dev.page_size)
	{
		return NVM_ERR_INVALID_ARG;
	}

	/* Get a pointer to the module hardware instance */
	Nvmctrl *const nvm_module = NVMCTRL;

	/* Check if the module is busy */
	if(!nvm_is_ready())
	{
		return NVM_BUSY;
	}

	/* Erase the page buffer before buffering new data */
	nvm_module->CTRLA.reg = NVM_COMMAND_PAGE_BUFFER_CLEAR | NVMCTRL_CTRLA_CMDEX_KEY;

	/* Check if the module is busy */
	while (!nvm_is_ready())
	{
		/* Force-wait for the buffer clear to complete */
	}

	/* Clear error flags */
	nvm_module->STATUS.reg = NVMCTRL_STATUS_MASK;

	uint32_t nvm_address = destination_address / 2;

	/* NVM _must_ be accessed as a series of 16-bit words, perform manual copy
	 * to ensure alignment */
	for(uint16_t i = 0; i < length; i += 2)
	{
		uint16_t data;

		/* Copy first byte of the 16-bit chunk to the temporary buffer */
		data = buffer[i];

		/* If we are not at the end of a write request with an odd byte count,
		 * store the next byte of data as well */
		if(i < (length - 1))
		{
			data |= (buffer[i + 1] << 8);
		}

		/* Store next 16-bit chunk to the NVM memory space */
		NVM_MEMORY[nvm_address++] = data;
	}

	/* If automatic page write mode is enable, then perform a manual NVM
	 * write when the length of data to be programmed is less than page size
	 */
	if((_nvm_dev.manual_page_write == 0) && (length < NVMCTRL_PAGE_SIZE))
	{
		return ((is_rww_eeprom) ? (drvr_nvmctrl_exe_cmd(NVM_COMMAND_RWWEE_WRITE_PAGE,destination_address)) : (drvr_nvmctrl_exe_cmd(NVM_COMMAND_WRITE_PAGE,destination_address)));
	}

	return NVM_SUCCESS;
}

/******************************************************************************
 * \brief Reads a number of bytes from a page in the NVM memory region.
 *
 * Reads a given number of bytes from a given page address in the NVM memory
 * space into a buffer.
 *
 * \param[in]  source_address  Source page address to read from
 * \param[out] buffer          Pointer to a buffer where the content of the read
 *                             page will be stored
 * \param[in]  length          Number of bytes in the page to read
 *
 * \return Status of the page read attempt.
 *
 * \retval NVM_SUCCESS               Requested NVM memory page was successfully
 *                                 read
 * \retval NVM_BUSY             NVM controller was busy when the operation
 *                                 was attempted
 * \retval NVM_ERR_BAD_ADDRESS  The requested address was outside the
 *                                 acceptable range of the NVM memory region or
 *                                 not aligned to the start of a page
 * \retval NVM_ERR_INVALID_ARG  The supplied read length was invalid
 ******************************************************************************/
drvr_nvmctrl_return_t drvr_nvmctrl_read_buffer(const uint32_t source_address, uint8_t * const buffer, const uint16_t length)
{
	/* Check if the source address is valid */
	if(source_address > ((uint32_t)_nvm_dev.page_size * _nvm_dev.number_of_pages))
	{
		if(source_address >= ((uint32_t)NVMCTRL_RWW_EEPROM_SIZE + NVMCTRL_RWW_EEPROM_ADDR) || source_address < NVMCTRL_RWW_EEPROM_ADDR)
		{
			return NVM_ERR_BAD_ADDRESS;
		}
	}

	/* Check if the read address is not aligned to the start of a page */
	if(source_address & (_nvm_dev.page_size - 1))
	{
		return NVM_ERR_BAD_ADDRESS;
	}

	/* Check if the write length is longer than an NVM page */
	if(length > _nvm_dev.page_size)
	{
		return NVM_ERR_INVALID_ARG;
	}

	/* Get a pointer to the module hardware instance */
	Nvmctrl *const nvm_module = NVMCTRL;

	/* Check if the module is busy */
	if(!nvm_is_ready())
	{
		return NVM_BUSY;
	}

	/* Clear error flags */
	nvm_module->STATUS.reg = NVMCTRL_STATUS_MASK;

	uint32_t page_address = source_address / 2;

	/* NVM _must_ be accessed as a series of 16-bit words, perform manual copy
	 * to ensure alignment */
	for(uint16_t i = 0; i < length; i += 2)
	{
		/* Fetch next 16-bit chunk from the NVM memory space */
		uint16_t data = NVM_MEMORY[page_address++];

		/* Copy first byte of the 16-bit chunk to the destination buffer */
		buffer[i] = (data & 0xFF);

		/* If we are not at the end of a read request with an odd byte count,
		 * store the next byte of data as well */
		if (i < (length - 1))
		{
			buffer[i + 1] = (data >> 8);
		}
	}

	return NVM_SUCCESS;
}

/******************************************************************************
 * \brief Erases a row in the NVM memory space.
 *
 * Erases a given row in the NVM memory region.
 *
 * \param[in] row_address  Address of the row to erase
 *
 * \return Status of the NVM row erase attempt.
 *
 * \retval NVM_SUCCESS               Requested NVM memory row was successfully
 *                                 erased
 * \retval NVM_BUSY             NVM controller was busy when the operation
 *                                 was attempted
 * \retval NVM_ERR_BAD_ADDRESS  The requested row address was outside the
 *                                 acceptable range of the NVM memory region or
 *                                 not aligned to the start of a row
 * \retval NVM_ABORTED          NVM erased error
******************************************************************************/
drvr_nvmctrl_return_t drvr_nvmctrl_erase_row(const uint32_t row_address)
{
		uint8_t is_rww_eeprom = 0;

	/* Check if the row address is valid */
	if(row_address > ((uint32_t)_nvm_dev.page_size * _nvm_dev.number_of_pages))
	{
		if(row_address >= ((uint32_t)NVMCTRL_RWW_EEPROM_SIZE + NVMCTRL_RWW_EEPROM_ADDR) || row_address < NVMCTRL_RWW_EEPROM_ADDR)
		{
			return NVM_ERR_BAD_ADDRESS;
		}
		is_rww_eeprom = 1;
	}

	/* Check if the address to erase is not aligned to the start of a row */
	if(row_address & ((_nvm_dev.page_size * NVMCTRL_ROW_PAGES) - 1))
	{
		return NVM_ERR_BAD_ADDRESS;
	}

	/* Get a pointer to the module hardware instance */
	Nvmctrl *const nvm_module = NVMCTRL;

	/* Check if the module is busy */
	if(!nvm_is_ready())
	{
		return NVM_BUSY;
	}

	/* Clear error flags */
	nvm_module->STATUS.reg = NVMCTRL_STATUS_MASK;

	/* Set address and command */
	nvm_module->ADDR.reg  = (uintptr_t)&NVM_MEMORY[row_address / 4];

	nvm_module->CTRLA.reg = ((is_rww_eeprom) ? (NVM_COMMAND_RWWEE_ERASE_ROW | NVMCTRL_CTRLA_CMDEX_KEY) : (NVM_COMMAND_ERASE_ROW | NVMCTRL_CTRLA_CMDEX_KEY));

	while (!nvm_is_ready());

	/* There existed error in NVM erase operation */
	if((nvm_error_t)(nvm_module->STATUS.reg & NVM_ERRORS_MASK) != NVM_ERROR_NONE)
	{
		return NVM_ABORTED;
	}

	return NVM_SUCCESS;
}

/******************************************************************************
 * \brief Reads the parameters of the NVM controller.
 *
 * Retrieves the page size, number of pages, and other configuration settings
 * of the NVM region.
 *
 * \param[out] parameters    Parameter structure, which holds page size and
 *                           number of pages in the NVM memory
******************************************************************************/
void drvr_nvmctrl_get_parameters(nvm_parameters_t * const parameters)
{
	/* Get a pointer to the module hardware instance */
	Nvmctrl *const nvm_module = NVMCTRL;

	/* Clear error flags */
	nvm_module->STATUS.reg = NVMCTRL_STATUS_MASK;

	/* Read out from the PARAM register */
	uint32_t param_reg = nvm_module->PARAM.reg;

	/* Mask out page size exponent and convert to a number of bytes */
	parameters->page_size =
			8 << ((param_reg & NVMCTRL_PARAM_PSZ_Msk) >> NVMCTRL_PARAM_PSZ_Pos);

	/* Mask out number of pages count */
	parameters->nvm_number_of_pages =
			(param_reg & NVMCTRL_PARAM_NVMP_Msk) >> NVMCTRL_PARAM_NVMP_Pos;

	/* Mask out rwwee number of pages count */
	parameters->rww_eeprom_number_of_pages =
			(param_reg & NVMCTRL_PARAM_RWWEEP_Msk) >> NVMCTRL_PARAM_RWWEEP_Pos;

	/* Read the current EEPROM fuse value from the USER row */
	uint16_t eeprom_fuse_value =
			(NVM_USER_MEMORY[NVMCTRL_FUSES_EEPROM_SIZE_Pos / 16] &
			NVMCTRL_FUSES_EEPROM_SIZE_Msk) >> NVMCTRL_FUSES_EEPROM_SIZE_Pos;

	/* Translate the EEPROM fuse byte value to a number of NVM pages */
	if (eeprom_fuse_value == 7) {
		parameters->eeprom_number_of_pages = 0;
	}
	else {
		parameters->eeprom_number_of_pages =
				NVMCTRL_ROW_PAGES << (6 - eeprom_fuse_value);
	}

	/* Read the current BOOTSZ fuse value from the USER row */
	uint16_t boot_fuse_value =
			(NVM_USER_MEMORY[NVMCTRL_FUSES_BOOTPROT_Pos / 16] &
			NVMCTRL_FUSES_BOOTPROT_Msk) >> NVMCTRL_FUSES_BOOTPROT_Pos;

	/* Translate the BOOTSZ fuse byte value to a number of NVM pages */
	if (boot_fuse_value == 7) {
		parameters->bootloader_number_of_pages = 0;
	}
	else {
		parameters->bootloader_number_of_pages =
				NVMCTRL_ROW_PAGES << (7 - boot_fuse_value);
	}
}

/******************************************************************************
 * \brief Checks whether the page region is locked.
 *
 * Extracts the region to which the given page belongs and checks whether
 * that region is locked.
 *
 * \param[in] page_number    Page number to be checked
 *
 * \return Page lock status.
 *
 * \retval 1              Page is locked
 * \retval 0             Page is not locked
 *
******************************************************************************/
uint8_t drvr_nvmctrl_is_page_locked(const uint16_t page_number)
{
	uint16_t pages_in_region;
	uint16_t region_number;

	/* Get a pointer to the module hardware instance */
	Nvmctrl *const nvm_module = NVMCTRL;

	/* Get number of pages in a region */
	pages_in_region = _nvm_dev.number_of_pages / 16;

	/* Get region for given page */
	region_number = page_number / pages_in_region;

	return !(nvm_module->LOCK.reg & (1 << region_number));
}

/******************************************************************************
 * \brief Checks if the NVM controller is ready to accept a new command.
 *
 * Checks the NVM controller to determine if it is currently busy execution an
 * operation, or ready for a new command.
 *
 * \return Busy state of the NVM controller.
 *
 * \retval 1   If the hardware module is ready for a new command
 * \retval 0  If the hardware module is busy executing a command
 *
******************************************************************************/
inline uint8_t nvm_is_ready(void)
{
	/* Get a pointer to the module hardware instance */
	Nvmctrl *const nvm_module = NVMCTRL;

	return nvm_module->INTFLAG.reg & NVMCTRL_INTFLAG_READY;
}
