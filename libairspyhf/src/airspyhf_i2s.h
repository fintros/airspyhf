/*
Copyright (c) 2024, Alexey Spirkov <dev@alsp.net>
Copyright (c) 2024, Youssef Touil <youssef@airspy.com>

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

#ifndef __AIRSPYHF__I2S_H__
#define __AIRSPYHF__I2S_H__

#ifndef _WIN32

#include "airspyhf_transport.h"
#include "airspyhf_commands.h"
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <errno.h>

static const char airspyhf_i2s_device[] = "/dev/airspy0";

#define TRANSFER_LIST_DEPTH 32

#define AIRSPY_IOCTL_MAX_DATA_SIZE 128

/// The following block can be included with #include "airspy-control.h" - moved here to reduce number of dependencies
typedef struct airspy_ioctl
{
	uint16_t data_len;
	uint8_t data[AIRSPY_IOCTL_MAX_DATA_SIZE - sizeof(uint16_t)];
} airspy_ioctl_t;

typedef struct airspy_ioctl_message
{
	uint64_t tx;
	uint64_t rx;
} airspy_ioctl_message_t;
/// end of block

typedef struct i2s_transport_data
{
    FILE *i2s_dev_file;
    pthread_t rx_thread;
    bool rx_thread_exit;
    pthread_mutex_t rx_condition_mutex;
    pthread_cond_t rx_condition;
    pthread_mutex_t rx_list_mutex;
    airspyhf_ll_transfer_t *circ_list[TRANSFER_LIST_DEPTH];
    int tl_head;
    int tl_tail;
} i2s_transport_data_t;

int transfer_list_empty(i2s_transport_data_t *data)
{
    return data->tl_head == -1;
}

void transfer_list_item_push(i2s_transport_data_t *data, airspyhf_ll_transfer_t *transfer)
{
    pthread_mutex_lock(&data->rx_list_mutex);
    if (data->tl_tail == -1)
        data->tl_tail = data->tl_head = 0;
    else
        data->tl_tail++;

    if (data->tl_tail == TRANSFER_LIST_DEPTH)
        data->tl_tail = 0;

    data->circ_list[data->tl_tail] = transfer;

    pthread_mutex_unlock(&data->rx_list_mutex);
}

airspyhf_ll_transfer_t *transfer_list_item_pop(i2s_transport_data_t *data)
{
    airspyhf_ll_transfer_t *result = 0;
    pthread_mutex_lock(&data->rx_list_mutex);
    if (data->tl_head >= 0)
    {
        result = data->circ_list[data->tl_head];
        if (data->tl_head == data->tl_tail)
            data->tl_head = data->tl_tail = -1;
        else
        {
            data->tl_head++;
            if (data->tl_head == TRANSFER_LIST_DEPTH)
                data->tl_head = 0;
        }
    }
    pthread_mutex_unlock(&data->rx_list_mutex);

    return result;
}

// Driver returns data in reverse order - swap
#define SWAP_IQ 1

void *airspyhf_i2s_rx_thread_func(void *ptr)
{
    airspyhf_device_t *dev_handle = (airspyhf_device_t *)ptr;
    i2s_transport_data_t *data = (i2s_transport_data_t *)(dev_handle->transport->transport_data);

    for (;;)
    {
        while (!transfer_list_empty(data))
        {
            airspyhf_ll_transfer_t *transfer = transfer_list_item_pop(data);
            if (transfer)
            {
                size_t read_words = fread(transfer->buffer, sizeof(uint32_t), transfer->length / sizeof(uint32_t), data->i2s_dev_file);
#ifdef SWAP_IQ
                uint32_t *word_buffer = (uint32_t *)transfer->buffer;
                for (int i = 0; i < read_words; i++)
                {
                    uint32_t w = word_buffer[i];
                    w = (w << 16) | (w >> 16);
                    word_buffer[i] = w;
                }
#endif
                if (transfer->callback)
                    transfer->callback(transfer, read_words * sizeof(uint32_t));
            }
        }

        if (data->rx_thread_exit)
            break;

        // wait for wake-up
        pthread_mutex_lock(&data->rx_condition_mutex);
        pthread_cond_wait(&data->rx_condition, &data->rx_condition_mutex);
        pthread_mutex_unlock(&data->rx_condition_mutex);
    }

    return 0;
}

static inline void wake_rx_thread(i2s_transport_data_t *data)
{
    pthread_cond_signal(&data->rx_condition);
}

static int airspyhf_i2s_set_receiver_mode(airspyhf_device_t *dev_handle, receiver_mode_t value)
{
    i2s_transport_data_t *data = (i2s_transport_data_t *)(dev_handle->transport->transport_data);

    //  ToDo - add ioctl to the device
    if (value == RECEIVER_MODE_OFF)
    {
        if (data->rx_thread)
        {
            data->rx_thread_exit = 1;
            wake_rx_thread(data);
            pthread_join(data->rx_thread, NULL);
            data->rx_thread = 0;
        }
    }
    else
    {
        data->rx_thread_exit = 0;
        pthread_create(&data->rx_thread, NULL, airspyhf_i2s_rx_thread_func, (void *)dev_handle);
    }

    return 0;
}

/////////////////////////////////////////////////////////////
///// I2S transport layer
/////////////////////////////////////////////////////////////

int i2s_transport_device_open(airspyhf_device_t *dev_handle, uint64_t serial_number_val, int fd)
{
    i2s_transport_data_t *data = (i2s_transport_data_t *)(dev_handle->transport->transport_data);

    // already opened?
    if (data->i2s_dev_file)
        return AIRSPYHF_ERROR;

    data->i2s_dev_file = fopen(airspyhf_i2s_device, "rb");
    if (data->i2s_dev_file)
        return AIRSPYHF_SUCCESS;

    return AIRSPYHF_ERROR;
}

void i2s_transport_reset(airspyhf_device_t *dev_handle)
{
    // nothing to do
}

void i2s_transport_stop(airspyhf_device_t *dev_handle)
{
    // kill thread if exists
    airspyhf_i2s_set_receiver_mode(dev_handle, RECEIVER_MODE_OFF);
}

void i2s_transport_device_close(airspyhf_device_t *dev_handle)
{
    i2s_transport_data_t *data = (i2s_transport_data_t *)(dev_handle->transport->transport_data);
    if (data->i2s_dev_file)
        fclose(data->i2s_dev_file);
    data->i2s_dev_file = NULL;
}

int i2s_transport_transfer_control(airspyhf_device_t *dev_handle,
                                   uint8_t bRequest,
                                   uint16_t wValue,
                                   uint16_t wIndex,
                                   unsigned char *data,
                                   uint16_t wLength)

{
    int result = 0;
    i2s_transport_data_t *tdata = (i2s_transport_data_t *)(dev_handle->transport->transport_data);

    if (bRequest == AIRSPYHF_RECEIVER_MODE)
        result = airspyhf_i2s_set_receiver_mode(dev_handle, wValue);

    if (!tdata->i2s_dev_file)
        return AIRSPYHF_ERROR;

    int fd = fileno(tdata->i2s_dev_file);

    airspy_ioctl_t transferOut;
    airspy_ioctl_t transferIn;
    transferOut.data_len = wLength;
    *(uint16_t *)(&transferOut.data[0]) = wValue;
    *(uint16_t *)(&transferOut.data[2]) = wIndex;

    airspy_ioctl_message_t ioctl_message;
    ioctl_message.tx = (uint64_t)&transferOut;
    ioctl_message.rx = (uint64_t)&transferIn;

    switch (bRequest)
    {
    case AIRSPYHF_RECEIVER_MODE:
    case AIRSPYHF_SET_FREQ:
    case AIRSPYHF_SET_SAMPLERATE:
    case AIRSPYHF_CONFIG_WRITE:
    case AIRSPYHF_SET_AGC:
    case AIRSPYHF_SET_AGC_THRESHOLD:
    case AIRSPYHF_SET_ATT:
    case AIRSPYHF_SET_LNA:
    case AIRSPYHF_SET_VCTCXO_CALIBRATION:
    case AIRSPYHF_SET_FRONTEND_OPTIONS:
    case AIRSPYHF_SET_BIAS_TEE:
        if (wLength > (sizeof(transferOut.data) - 4))
            wLength = sizeof(transferOut.data) - 4; // should never happens - just guard

        if (wLength)
            memcpy(&transferOut.data[4], data, wLength);

        // some lower ioctls seems to cause issues - at least #2 does not passed to the driver - it is workaround - to check
        result = ioctl(fd, bRequest + 0x100, &ioctl_message);
        break;
    case AIRSPYHF_GET_SERIALNO_BOARDID:
    case AIRSPYHF_SET_USER_OUTPUT:
    case AIRSPYHF_GET_VERSION_STRING:
    case AIRSPYHF_GET_SAMPLERATES:
    case AIRSPYHF_CONFIG_READ:
    case AIRSPYHF_GET_SAMPLERATE_ARCHITECTURES:
    case AIRSPYHF_GET_FILTER_GAIN:
    case AIRSPYHF_GET_FREQ_DELTA:
    case AIRSPYHF_GET_ATT_STEPS:
    case AIRSPYHF_GET_BIAS_TEE_COUNT:
    case AIRSPYHF_GET_BIAS_TEE_NAME:
    default:
        // some lower ioctls seems to cause issues - at least #2 does not passed to the driver - it is workaround - to check
        result = ioctl(fd, bRequest + 0x100, &ioctl_message);
        if (result > 0)
        {
            if (wLength > sizeof(transferIn.data))
                wLength = sizeof(transferIn.data);

            if (data && wLength)
                memcpy(data, &transferIn.data[0], wLength);
        }
    }

    return result;
}

int i2s_transport_transfer_cancel(airspyhf_device_t *dev_handle, airspyhf_ll_transfer_t *transfer)
{

    return AIRSPYHF_SUCCESS;
}

int i2s_transport_transfer_submit(airspyhf_device_t *dev_handle, airspyhf_ll_transfer_t *transfer)
{
    i2s_transport_data_t *data = (i2s_transport_data_t *)(dev_handle->transport->transport_data);
    transfer_list_item_push(data, transfer);
    wake_rx_thread(data);

    return AIRSPYHF_SUCCESS;
}

airspyhf_ll_transfer_t *i2s_transport_transfer_alloc(airspyhf_device_t *dev_handle)
{
    airspyhf_ll_transfer_t *transfer = (airspyhf_ll_transfer_t *)malloc(sizeof(airspyhf_ll_transfer_t));
    if (!transfer)
        return 0;
    transfer->buffer = malloc(dev_handle->buffer_size);
    if (!transfer->buffer)
    {
        free(transfer);
        return 0;
    }
    transfer->length = dev_handle->buffer_size;
    transfer->transport_data = 0;

    return transfer;
}

int i2s_transport_transfer_prepare(airspyhf_device_t *dev_handle, airspyhf_ll_transfer_t *transfer, ll_transfer_cb_fn callback)
{
    transfer->callback = callback;
    return i2s_transport_transfer_submit(dev_handle, transfer);
}

void i2s_transport_transfer_free(airspyhf_device_t *dev_handle, airspyhf_ll_transfer_t *transfer)
{
    free(transfer->buffer);
    free(transfer);
}

void i2s_transport_exit(airspyhf_transport_t *transport)
{
}

int i2s_transport_handle_events(airspyhf_device_t *dev_handle, struct timeval *timeout)
{
    i2s_transport_data_t *data = (i2s_transport_data_t *)(dev_handle->transport->transport_data);
    wake_rx_thread(data);
    return AIRSPYHF_SUCCESS;
}

static int read_serial_no(FILE *f, uint64_t *serial)
{
    int result = 0;
    if (serial)
    {
        int fd = fileno(f);

        airspy_ioctl_t transferOut;
        airspy_ioctl_t transferIn;
        transferOut.data_len = sizeof(uint64_t);
        *(uint16_t *)(&transferOut.data[0]) = 0;
        *(uint16_t *)(&transferOut.data[2]) = 0;

        airspy_ioctl_message_t ioctl_message;
        ioctl_message.tx = (uint64_t)&transferOut;
        ioctl_message.rx = (uint64_t)&transferIn;

        result = ioctl(fd, AIRSPYHF_GET_SERIALNO_BOARDID + 0x100, &ioctl_message);
        if (result)
            memcpy(serial, transferIn.data, sizeof(uint64_t));
    }
    return result;
}

int i2s_transport_list_devices(airspyhf_transport_t *transport, uint64_t *serials, int count)
{
    i2s_transport_data_t *data = (i2s_transport_data_t *)(transport->transport_data);
    int i2s_exists = 1;

    if (!data->i2s_dev_file)
    {
        data->i2s_dev_file = fopen(airspyhf_i2s_device, "rb");
        if (data->i2s_dev_file)
        {
            if (count && serials)
                read_serial_no(data->i2s_dev_file, serials);

            fclose(data->i2s_dev_file);
            data->i2s_dev_file = 0;
            i2s_exists = 1;
        }
    }
    else
    {
        if (count && serials)
            read_serial_no(data->i2s_dev_file, serials);

        i2s_exists = 1;
    }

    if (i2s_exists)
        return 1;

    return 0;
}

void i2s_transport_transfer_set_buffer(airspyhf_ll_transfer_t *transfer, void *buffer)
{
    transfer->buffer = buffer;
}

int airspyhf_get_i2s_transport(airspyhf_transport_t *transport)
{
    transport->device_open = i2s_transport_device_open;
    transport->device_reset = i2s_transport_reset;
    transport->device_stop = i2s_transport_stop;
    transport->device_close = i2s_transport_device_close;
    transport->control_transfer = i2s_transport_transfer_control;
    transport->cancel_transfer = i2s_transport_transfer_cancel;
    transport->submit_transfer = i2s_transport_transfer_submit;
    transport->alloc_transfer = i2s_transport_transfer_alloc;
    transport->prepare_transfer = i2s_transport_transfer_prepare;
    transport->free_transfer = i2s_transport_transfer_free;
    transport->exit = i2s_transport_exit;
    transport->handle_events = i2s_transport_handle_events;
    transport->list_devices = i2s_transport_list_devices;
    transport->set_transfer_buffer = i2s_transport_transfer_set_buffer;
    transport->transport_data = malloc(sizeof(i2s_transport_data_t));
    if (transport->transport_data)
    {
        i2s_transport_data_t *data = (i2s_transport_data_t *)(transport->transport_data);
        data->i2s_dev_file = NULL;
        data->rx_thread = 0;
        data->rx_thread_exit = 0;
        pthread_mutex_init(&data->rx_condition_mutex, NULL);
        pthread_cond_init(&data->rx_condition, NULL);
        pthread_mutex_init(&data->rx_list_mutex, NULL);
        data->tl_head = -1;
        data->tl_tail = -1;

        return AIRSPYHF_SUCCESS;
    }
    else
        return AIRSPYHF_ERROR;
}

#define I2S_EXIST

#endif

#endif
