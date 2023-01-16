#ifndef UW_RECEIVER_H
#define UW_RECEIVER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Main Functions */
extern int32_t UW_rx_update( uint16_t rc_raw_command_arr[], size_t size_arr, uvos_rcvr_channelgroups_type_e rx_protocol );

#ifdef __cplusplus
}
#endif

#endif // UW_RECEIVER_H