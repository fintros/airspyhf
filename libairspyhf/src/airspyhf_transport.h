/*
Copyright (c) 2024, Youssef Touil <youssef@airspy.com>
Copyright (c) 2024, Alexey Spirkov <dev@alsp.net>

All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

        Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
        Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
        Neither the name of Airspy HF+ nor the names of its contributors may be used to endorse or promote products derived from this software
        without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef __AIRSPYHF__TRANSPORT_H__
#define __AIRSPYHF__TRANSPORT_H__

#include <stdint.h>

// forward declarations
struct airspyhf_device;
struct airspyhf_ll_transfer;
struct airspyhf_transport;
struct timeval;


typedef int (*ll_transfer_control_fn)(struct airspyhf_device* dev_handle,
                                     uint8_t bRequest,
                                     uint16_t wValue,
                                     uint16_t wIndex,
                                     unsigned char *data,
                                     uint16_t wLength);

typedef int (*device_open_fn)(struct airspyhf_device* dev_handle, uint64_t serial_number_val, int fd);
typedef int (*ll_transfer_submit_fn)(struct airspyhf_device* dev_handle, struct airspyhf_ll_transfer* transfer);
typedef int (*ll_transfer_cancel_fn)(struct airspyhf_device* dev_handle, struct airspyhf_ll_transfer* transfer);
typedef void (*device_default_fn)(struct airspyhf_device* dev_handle);
typedef void (*ll_transfer_cb_fn)(struct airspyhf_ll_transfer* transfer, int length);
typedef struct airspyhf_ll_transfer* (*ll_transfer_alloc_fn)(struct airspyhf_device* dev_handle);
typedef int (*ll_transfer_prepare_fn)(struct airspyhf_device* dev_handle, struct airspyhf_ll_transfer* transfer, ll_transfer_cb_fn callback);
typedef void (*ll_transfer_free_fn)(struct airspyhf_device* dev_handle, struct airspyhf_ll_transfer* transfer);
typedef void (*transport_exit_fn)(struct airspyhf_transport* transport);
typedef int (*device_handle_events_fn)(struct airspyhf_device* dev_handle, struct timeval *timeout);
typedef int (*list_devices_fn)(struct airspyhf_transport* transport, uint64_t *serials, int count);
typedef void (*ll_transfer_set_buffer_fn)(struct airspyhf_ll_transfer* transfer, void* buffer);

typedef struct airspyhf_transport
{
    list_devices_fn list_devices;
    device_open_fn device_open;
    device_default_fn device_reset;
    device_default_fn device_stop;
    device_default_fn device_close;
    ll_transfer_control_fn control_transfer;
    ll_transfer_submit_fn submit_transfer;
    ll_transfer_cancel_fn cancel_transfer;
    ll_transfer_alloc_fn alloc_transfer;
    ll_transfer_prepare_fn prepare_transfer;
    ll_transfer_free_fn free_transfer;
    transport_exit_fn exit;
    device_handle_events_fn handle_events;
    ll_transfer_set_buffer_fn set_transfer_buffer;  // needed for the trick with buffers swapping
    void* transport_data;
} airspyhf_transport_t;

#endif 