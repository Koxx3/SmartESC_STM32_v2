/*
	Copyright 2016 - 2019 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include <string.h>
#include "packet.h"
#include "crc.h"
#include "FreeRTOS.h"

/**
 * The latest update aims at achieving optimal re-synchronization in the
 * case if lost data, at the cost of some performance.
 */

// Defines
#define BUFFER_LEN				(PACKET_MAX_PL_LEN + 8)

// Private functions
static int try_decode_packet(unsigned char *buffer, unsigned int in_len,
		void(*process_func)(unsigned char *data, unsigned int len, PACKET_STATE_t * phandle), int *bytes_left, PACKET_STATE_t * phandle);

PACKET_STATE_t * packet_init(void (*s_func)(unsigned char *data, unsigned int len, port_str * port), void (*p_func)(unsigned char *data, unsigned int len, PACKET_STATE_t *phandle), port_str * port) {
	PACKET_STATE_t * ret = pvPortMalloc(sizeof(PACKET_STATE_t));
	memset(ret, 0, sizeof(PACKET_STATE_t));
	ret->send_func = s_func;
	ret->process_func = p_func;
	ret->port = port;
	ret->rx_buffer = pvPortMalloc(BUFFER_LEN);
	return ret;
}

void packet_free(PACKET_STATE_t * handle){
	vPortFree(handle->rx_buffer);
	vPortFree(handle);
}

void packet_reset(PACKET_STATE_t * state) {
	state->rx_read_ptr = 0;
	state->rx_write_ptr = 0;
	state->bytes_left = 0;
}

void packet_send_packet(unsigned char *send_buffer, unsigned int len, PACKET_STATE_t * handle) {
	if (len == 0 || len > PACKET_MAX_PL_LEN) {
		return;
	}

	uint8_t * data = send_buffer + PACKET_HEADER;

	int b_ind = 0;

	if (len <= 255) {
		send_buffer+=2;
		send_buffer[b_ind++] = 2;
		send_buffer[b_ind++] = len;
	} else if (len <= 65535) {
		send_buffer+=1;
		send_buffer[b_ind++] = 3;
		send_buffer[b_ind++] = len >> 8;
		send_buffer[b_ind++] = len & 0xFF;
	} else {
		send_buffer[b_ind++] = 4;
		send_buffer[b_ind++] = len >> 16;
		send_buffer[b_ind++] = (len >> 8) & 0x0F;
		send_buffer[b_ind++] = len & 0xFF;
	}

	b_ind += len;

	unsigned short crc = crc16(data, len);
	send_buffer[b_ind++] = (uint8_t)(crc >> 8);
	send_buffer[b_ind++] = (uint8_t)(crc & 0xFF);
	send_buffer[b_ind++] = 3;

	if (handle->send_func) {
		handle->send_func(send_buffer, b_ind, handle->port);
	}
}

/**
 * Call this function every millisecond. This is not strictly necessary
 * if the timeout is unimportant.
 */
void packet_timerfunc(void) {
	/*for (int i = 0;i < PACKET_HANDLERS;i++) {
		if (m_handler_states[i].rx_timeout) {
			m_handler_states[i].rx_timeout--;
		} else {
			packet_reset(i);
		}
	}*/
}

void packet_process_byte(uint8_t rx_data, PACKET_STATE_t * handle) {

	handle->rx_timeout = PACKET_RX_TIMEOUT;

	unsigned int data_len = handle->rx_write_ptr - handle->rx_read_ptr;

	// Out of space (should not happen)
	if (data_len >= BUFFER_LEN) {
		handle->rx_write_ptr = 0;
		handle->rx_read_ptr = 0;
		handle->bytes_left = 0;
		handle->rx_buffer[handle->rx_write_ptr++] = rx_data;
		return;
	}

	// Everything has to be aligned, so shift buffer if we are out of space.
	// (as opposed to using a circular buffer)
	if (handle->rx_write_ptr >= BUFFER_LEN) {
		memmove(handle->rx_buffer,
				handle->rx_buffer + handle->rx_read_ptr,
				data_len);

		handle->rx_read_ptr = 0;
		handle->rx_write_ptr = data_len;
	}

	handle->rx_buffer[handle->rx_write_ptr++] = rx_data;
	data_len++;

	if (handle->bytes_left > 1) {
		handle->bytes_left--;
		return;
	}

	// Try decoding the packet at various offsets until it succeeds, or
	// until we run out of data.
	for (;;) {
		int res = try_decode_packet(handle->rx_buffer + handle->rx_read_ptr,
				data_len, handle->process_func, &handle->bytes_left, handle);

		// More data is needed
		if (res == -2) {
			break;
		}

		if (res > 0) {
			data_len -= res;
			handle->rx_read_ptr += res;
		} else if (res == -1) {
			// Something went wrong. Move pointer forward and try again.
			handle->rx_read_ptr++;
			data_len--;
		}
	}

	// Nothing left, move pointers to avoid memmove
	if (data_len == 0) {
		handle->rx_read_ptr = 0;
		handle->rx_write_ptr = 0;
	}
}

/**
 * Try if it is possible to decode a packet from a buffer.
 *
 * @param buffer
 * The buffer to try from
 *
 * @param in_len
 * The length of the buffer
 *
 * @param process_func
 * Call this function with the decoded packet on success. Set to null
 * to disable.
 *
 * @param bytes_left
 * This many additional bytes are required to tell more about the packet.
 *
 * @return
 * >0: Success, number of bytes decoded from buffer (not payload length)
 * -1: Invalid structure
 * -2: OK so far, but not enough data
 */
static int try_decode_packet(unsigned char *buffer, unsigned int in_len,
		void(*process_func)(unsigned char *data, unsigned int len, PACKET_STATE_t * phandle), int *bytes_left, PACKET_STATE_t * phandle) {
	*bytes_left = 0;

	if (in_len == 0) {
		*bytes_left = 1;
		return -2;
	}

	bool is_len_8b = buffer[0] == 2;
	unsigned int data_start = buffer[0];

#if PACKET_MAX_PL_LEN > 255
	bool is_len_16b = buffer[0] == 3;
#else
#define is_len_16b false
#endif

#if PACKET_MAX_PL_LEN > 65535
	bool is_len_24b = buffer[0] == 4;
#else
#define is_len_24b false
#endif

	// No valid start byte
	if (!is_len_8b && !is_len_16b && !is_len_24b) {
		return -1;
	}

	// Not enough data to determine length
	if (in_len < data_start) {
		*bytes_left = data_start - in_len;
		return -2;
	}

	unsigned int len = 0;

	if (is_len_8b) {
		len = (unsigned int)buffer[1];

		// No support for zero length packets
		if (len < 1) {
			return -1;
		}
	} else if (is_len_16b) {
		len = (unsigned int)buffer[1] << 8 | (unsigned int)buffer[2];

		// A shorter packet should use less length bytes
		if (len < 255) {
			return -1;
		}
	} else if (is_len_24b) {
		len = (unsigned int)buffer[1] << 16 |
				(unsigned int)buffer[2] << 8 |
				(unsigned int)buffer[3];

		// A shorter packet should use less length bytes
		if (len < 65535) {
			return -1;
		}
	}

	// Too long packet
	if (len > PACKET_MAX_PL_LEN) {
		return -1;
	}

	// Need more data to determine rest of packet
	if (in_len < (len + data_start + 3)) {
		*bytes_left = (len + data_start + 3) - in_len;
		return -2;
	}

	// Invalid stop byte
	if (buffer[data_start + len + 2] != 3) {
		return -1;
	}

	unsigned short crc_calc = crc16(buffer + data_start, len);
	unsigned short crc_rx = (unsigned short)buffer[data_start + len] << 8
							| (unsigned short)buffer[data_start + len + 1];

	if (crc_calc == crc_rx) {
		if (process_func) {
			process_func(buffer + data_start, len, phandle);
		}

		return len + data_start + 3;
	} else {
		return -1;
	}
}
