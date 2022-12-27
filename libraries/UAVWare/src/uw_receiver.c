#include <uvos.h>

/**
 * @brief Reads all input channels from the specified driver
 * @param[in] rc_raw_command_arr[] 0 based command array to populate
 * @param[in] num_channels number of channels to populate
 * @returns Error code else 0 if successful
 *  @retval UVOS_RCVR_TIMEOUT indicates a failsafe or timeout from that channel
 *  @retval UVOS_RCVR_INVALID invalid channel for this driver (usually out of range supported)
 *  @retval UVOS_RCVR_NODRIVER driver was not initialized
 */
int32_t UW_rx_update( uint16_t rc_raw_command_arr[], size_t num_channels, uvos_rcvr_channelgroups_type_e rx_protocol )
{
  extern uint32_t uvos_rcvr_group_map[];

  /* Publicly facing UVOS_RCVR_Read API uses channel 1 for first channel */
  for ( uint32_t i = 1; i <= num_channels; ++i ) {
    int32_t raw_command = UVOS_RCVR_Read( uvos_rcvr_group_map[ rx_protocol ], i );
    if ( ( raw_command == UVOS_RCVR_NODRIVER ) || ( raw_command == UVOS_RCVR_INVALID ) || ( raw_command == UVOS_RCVR_TIMEOUT ) ) {
      return raw_command;
    } else {
      rc_raw_command_arr[ i - 1 ] = ( uint16_t )raw_command;
    }
  }
  return 0;
}