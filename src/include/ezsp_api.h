/*
 * ezsp_api.h
 *
 *  Created on: Jan 14, 2016
 *      Author: titan
 */

#ifndef EZSP_API_H_
#define EZSP_API_H_
#define PACKED __attribute__((packed))

typedef struct
{
	uint8_t sequence_byte;
	uint8_t control_byte;
	uint8_t frame_id;

} ezsp_header_t;
ezsp_header_t ezsp_header;

typedef struct PACKED
{
	uint16_t options_masks;
} ember_apps_options_t;
typedef struct PACKED
{
	uint16_t profile_id;
	uint16_t cluster_id;
	uint8_t source_endpoint;
	uint8_t destination_endpoint;
	ember_apps_options_t apps_options;
	uint16_t group_id;
	uint8_t sequence;
} ember_apps_frame_t;

typedef struct PACKED
{
	ezsp_header_t header;
	uint8_t message_type;
	uint16_t destination;
	ember_apps_frame_t apps_frame;
	uint8_t message_tag;
	uint8_t message_length;
	uint8_t message[10];
}send_unicast_t;
send_unicast_t send_unicast;

typedef struct
{
	ezsp_header_t header;
	uint16_t destination_node;
	ember_apps_frame_t apps_frame;
	uint8_t radius;
	uint8_t message_tag;
	uint8_t message_length;
	uint8_t message[10];

} send_broadcast_t;
send_broadcast_t send_broadcast;

typedef struct
{
	ezsp_header_t header;
	uint8_t desired_protocol_version;
} version_command_t;
version_command_t version_command;

typedef struct
	PACKED
	{
		ezsp_header_t header;
		uint16_t bitmask;
		uint8_t preconfigured_key[16];
		uint8_t network_key[16];
		uint8_t sequence_number;
		uint64_t trust_center_eui64;
	} initial_security_state_t;
	initial_security_state_t initial_security_state;

	typedef struct
	{
		ezsp_header_t header;
	} get_current_security_state_t;

	get_current_security_state_t get_current_security_state;

	typedef struct
	{
		ezsp_header_t header;
	} network_init_cmd_t;
	network_init_cmd_t network_init_cmd;

	typedef struct
	{
		ezsp_header_t header;
		uint8_t channel;
	} set_channel_cmd_t;
	set_channel_cmd_t set_channel_cmd;

	typedef struct
	{
		ezsp_header_t header;
	} get_network_params_cmd_t;
	get_network_params_cmd_t get_network_params_cmd;

	typedef struct PAKCED
	{
		//ezsp_header_t header;
		uint8_t extendedPanId[8];
		uint16_t panId;
		uint8_t radioTxPower;
		uint8_t radioChannel;
		uint8_t joinMethod;
//	uint16_t nwkManagerId;
//	uint8_t nwkUpdateId;
//	uint32_t channels;
	} ember_network_parameters_t;
//ember_network_parameters_t ember_network_parameters;

	typedef struct
		PACKED {
			ezsp_header_t header;
			ember_network_parameters_t net_params;
		} form_network_t;
		form_network_t form_network;

		typedef struct
			PACKED {
				ezsp_header_t header;
				uint8_t node_type;
				ember_network_parameters_t net_params;
			} join_network_t;

			join_network_t join_network;

			typedef struct
			{
				ezsp_header_t header;
				uint8_t duration;
			} permit_joining_t;

			permit_joining_t permit_joining;

			typedef struct
			{
				ezsp_header_t header;
			} network_state_t;

			network_state_t network_state;

			typedef struct
			{
				ezsp_header_t header;
				uint8_t scan_type;
				uint32_t channel_mask;
				uint8_t duration;
			} start_scan_t;
			start_scan_t start_scan;

			typedef struct
			{
				ezsp_header_t header;
			} stop_scan_t;

			stop_scan_t stop_scan;

			typedef struct
			{
				ezsp_header_t header;
			} callback_cmd_t;

			callback_cmd_t callback_cmd;

#endif /* EZSP_API_H_ */
