#ifndef UVOS_DEBUGLOG_H
#define UVOS_DEBUGLOG_H


/**
 * @brief Initialize the log facility
 */
void UVOS_DEBUGLOG_Initialize();

/**
 * @brief Enables or Disables logging globally
 * @param[in] enable or disable logging
 */
void UVOS_DEBUGLOG_Enable(uint8_t enabled);

/**
 * @brief Write a debug log entry with a uavobject
 * @param[in] objectid
 * @param[in] instanceid
 * @param[in] instanceid
 * @param[in] size of object
 * @param[in] data buffer
 */
void UVOS_DEBUGLOG_UAVObject(uint32_t objid, uint16_t instid, size_t size, uint8_t *data);

/**
 * @brief Write a debug log entry with text
 * @param[in] format - as in printf
 * @param[in] variable arguments for printf
 * @param...
 */
void UVOS_DEBUGLOG_Printf(char *format, ...);

/**
 * @brief Load one object instance from the filesystem
 * @param[out] buffer where to store the uavobject
 * @param[in] log entry from which flight
 * @param[in] log entry sequence number
 * @return 0 if success or error code
 * @retval -1 if fs_id is not a valid filesystem instance
 * @retval -2 if failed to start transaction
 * @retval -3 if object not found in filesystem
 * @retval -4 if object size in filesystem does not exactly match buffer size
 * @retval -5 if reading the object data from flash fails
 */
int32_t UVOS_DEBUGLOG_Read(void *buffer, uint16_t flight, uint16_t inst);

/**
 * @brief Retrieve run time info of logging system
 * @param[out] current flight number
 * @param[out] next entry number
 * @param[out] free slots in filesystem
 * @param[out] used slots in filesystem
 */
void UVOS_DEBUGLOG_Info(uint16_t *flight, uint16_t *entry, uint16_t *free, uint16_t *used);

/**
 * @brief Format entire flash memory!!!
 */
void UVOS_DEBUGLOG_Format(void);

#endif // ifndef UVOS_DEBUGLOG_H
