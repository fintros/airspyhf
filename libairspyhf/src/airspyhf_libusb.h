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

#ifndef __AIRSPYHF__LIBUSB_H__
#define __AIRSPYHF__LIBUSB_H__

#include "airspyhf_transport.h"

#include <libusb.h>

typedef struct libusb_transport_data
{
    libusb_context *usb_context;
    libusb_device_handle *usb_device;
    struct airspyhf_device *dev_handle;
} libusb_transport_data_t;

static const uint16_t airspyhf_usb_vid = 0x03EB;
static const uint16_t airspyhf_usb_pid = 0x800C;

#define LIBUSB_CTRL_TIMEOUT_MS (500)

#define STR_PREFIX_SERIAL_AIRSPYHF_SIZE (12)
static const char str_prefix_serial_airspyhf[STR_PREFIX_SERIAL_AIRSPYHF_SIZE] =
    {'A', 'I', 'R', 'S', 'P', 'Y', 'H', 'F', ' ', 'S', 'N', ':'};

static int libusb_get_endpoint_by_request(uint8_t bRequest)
{
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
        return LIBUSB_ENDPOINT_OUT;
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
        return LIBUSB_ENDPOINT_IN;
    }
}

static int libusb_transport_transfer_control(struct airspyhf_device *dev_handle,
                                             uint8_t bRequest,
                                             uint16_t wValue,
                                             uint16_t wIndex,
                                             unsigned char *data,
                                             uint16_t wLength)
{
    libusb_transport_data_t *transport_data = (libusb_transport_data_t *)dev_handle->transport->transport_data;

    return libusb_control_transfer(
        transport_data->usb_device,
        libusb_get_endpoint_by_request(bRequest) | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
        bRequest,
        wValue,
        wIndex,
        data,
        wLength,
        LIBUSB_CTRL_TIMEOUT_MS);
}

static void airspyhf_open_device(airspyhf_device_t *device,
                                 int *ret,
                                 uint16_t vid,
                                 uint16_t pid,
                                 uint64_t serial_number_val)
{
    int i;
    int result;
    libusb_device_handle **libusb_dev_handle;
    int serial_number_len;
    libusb_device_handle *dev_handle;
    libusb_device *dev;
    libusb_device **devices = NULL;
    libusb_transport_data_t *transport_data = (libusb_transport_data_t *)device->transport->transport_data;

    ssize_t cnt;
    int serial_descriptor_index;
    struct libusb_device_descriptor device_descriptor;
    unsigned char serial_number[AIRSPYHF_SERIAL_SIZE + 1];

    libusb_dev_handle = &transport_data->usb_device;
    *libusb_dev_handle = NULL;

    cnt = libusb_get_device_list(transport_data->usb_context, &devices);
    if (cnt < 0)
    {
        *ret = AIRSPYHF_ERROR;
        return;
    }

    i = 0;
    while ((dev = devices[i++]) != NULL)
    {
        libusb_get_device_descriptor(dev, &device_descriptor);

        if ((device_descriptor.idVendor == vid) &&
            (device_descriptor.idProduct == pid))
        {
            if (serial_number_val != SERIAL_NUMBER_UNUSED)
            {
                serial_descriptor_index = device_descriptor.iSerialNumber;
                if (serial_descriptor_index > 0)
                {
                    if (libusb_open(dev, libusb_dev_handle) != 0)
                    {
                        *libusb_dev_handle = NULL;
                        continue;
                    }
                    dev_handle = *libusb_dev_handle;
                    serial_number_len = libusb_get_string_descriptor_ascii(dev_handle,
                                                                           serial_descriptor_index,
                                                                           serial_number,
                                                                           sizeof(serial_number));

                    if (serial_number_len == AIRSPYHF_SERIAL_SIZE && !memcmp(str_prefix_serial_airspyhf, serial_number, STR_PREFIX_SERIAL_AIRSPYHF_SIZE))
                    {
                        uint64_t serial = SERIAL_NUMBER_UNUSED;
                        // use same code to determine device's serial number as in airspyhf_list_devices()
                        {
                            char *start, *end;
                            serial_number[AIRSPYHF_SERIAL_SIZE] = 0;
                            start = (char *)(serial_number + STR_PREFIX_SERIAL_AIRSPYHF_SIZE);
                            end = NULL;
                            serial = strtoull(start, &end, 16);
                        }
                        if (serial == serial_number_val)
                        {
                            result = libusb_set_configuration(dev_handle, 1);
                            if (result != 0)
                            {
                                libusb_close(dev_handle);
                                *libusb_dev_handle = NULL;
                                continue;
                            }
                            result = libusb_claim_interface(dev_handle, 0);
                            if (result != 0)
                            {
                                libusb_close(dev_handle);
                                *libusb_dev_handle = NULL;
                                continue;
                            }

                            result = libusb_set_interface_alt_setting(dev_handle, 0, 1);
                            if (result != 0)
                            {
                                libusb_close(dev_handle);
                                *libusb_dev_handle = NULL;
                                continue;
                            }

                            break;
                        }
                        else
                        {
                            libusb_close(dev_handle);
                            *libusb_dev_handle = NULL;
                            continue;
                        }
                    }
                    else
                    {
                        libusb_close(dev_handle);
                        *libusb_dev_handle = NULL;
                        continue;
                    }
                }
            }
            else
            {
                if (libusb_open(dev, libusb_dev_handle) == 0)
                {
                    dev_handle = *libusb_dev_handle;
                    result = libusb_set_configuration(dev_handle, 1);
                    if (result != 0)
                    {
                        libusb_close(dev_handle);
                        *libusb_dev_handle = NULL;
                        continue;
                    }
                    result = libusb_claim_interface(dev_handle, 0);
                    if (result != 0)
                    {
                        libusb_close(dev_handle);
                        *libusb_dev_handle = NULL;
                        continue;
                    }

                    result = libusb_set_interface_alt_setting(dev_handle, 0, 1);
                    if (result != 0)
                    {
                        libusb_close(dev_handle);
                        *libusb_dev_handle = NULL;
                        continue;
                    }
                    break;
                }
            }
        }
    }
    libusb_free_device_list(devices, 1);

    dev_handle = transport_data->usb_device;
    if (dev_handle == NULL)
    {
        *ret = AIRSPYHF_ERROR;
        return;
    }

    *ret = AIRSPYHF_SUCCESS;
    return;
}

static void airspyhf_open_device_fd(airspyhf_device_t *device,
                                    int *ret,
                                    int fd)
{
    int result = -1;

    libusb_transport_data_t *transport_data = (libusb_transport_data_t *)device->transport->transport_data;

#ifdef __ANDROID__
    result = libusb_wrap_sys_device(transport_data->usb_context, (intptr_t)fd, &transport_data->usb_device);
#else
    transport_data->usb_device = NULL;
    *ret = AIRSPYHF_UNSUPPORTED;
    return;
#endif

    if (result != 0 || transport_data->usb_device == NULL)
    {
        *ret = AIRSPYHF_ERROR;
        return;
    }

    result = libusb_set_configuration(transport_data->usb_device, 1);
    if (result != 0)
    {
        libusb_close(transport_data->usb_device);
        transport_data->usb_device = NULL;
        *ret = AIRSPYHF_ERROR;
        return;
    }

    result = libusb_claim_interface(transport_data->usb_device, 0);
    if (result != 0)
    {
        libusb_close(transport_data->usb_device);
        transport_data->usb_device = NULL;
        *ret = AIRSPYHF_ERROR;
        return;
    }

    result = libusb_set_interface_alt_setting(transport_data->usb_device, 0, 1);
    if (result != 0)
    {
        libusb_close(transport_data->usb_device);
        transport_data->usb_device = NULL;
        *ret = AIRSPYHF_ERROR;
        return;
    }

    *ret = AIRSPYHF_SUCCESS;
    return;
}

static int libusb_transport_device_open(struct airspyhf_device *dev_handle, uint64_t serial_number_val, int fd)
{
    int result;

    if (fd == FILE_DESCRIPTOR_UNUSED)
    {
        airspyhf_open_device(dev_handle,
                             &result,
                             airspyhf_usb_vid,
                             airspyhf_usb_pid,
                             serial_number_val);
    }
    else
    {
        airspyhf_open_device_fd(dev_handle,
                                &result,
                                fd);
    }
    return result;
}

static int libusb_transport_transfer_submit(struct airspyhf_device *dev_handle, struct airspyhf_ll_transfer *transfer)
{
    struct libusb_transfer *usb_transfer = (struct libusb_transfer *)transfer->transport_data;
    if (libusb_submit_transfer(usb_transfer) != 0)
        return AIRSPYHF_ERROR;
    return AIRSPYHF_SUCCESS;
}

static int libusb_transport_transfer_cancel(struct airspyhf_device *dev_handle, struct airspyhf_ll_transfer *transfer)
{
    struct libusb_transfer *usb_transfer = (struct libusb_transfer *)transfer->transport_data;
    if (libusb_cancel_transfer(usb_transfer))
        return AIRSPYHF_ERROR;
    return AIRSPYHF_SUCCESS;
}

static void libusb_transport_device_close(struct airspyhf_device *dev_handle)
{
}

static struct airspyhf_ll_transfer *libusb_transport_transfer_alloc(struct airspyhf_device *dev_handle)
{
    libusb_transport_data_t *transport_data = (libusb_transport_data_t *)dev_handle->transport->transport_data;
    struct airspyhf_ll_transfer *transfer = (struct airspyhf_ll_transfer *)malloc(sizeof(struct airspyhf_ll_transfer));
    if (!transfer)
        return 0;
    struct libusb_transfer *libusb_transfer = libusb_alloc_transfer(0);
    if (!libusb_transfer)
    {
        free(transfer);
        return 0;
    }

    transfer->transport_data = libusb_transfer;
    transfer->buffer = malloc(dev_handle->buffer_size);
    transfer->length = dev_handle->buffer_size;

    libusb_fill_bulk_transfer(
        libusb_transfer,
        transport_data->usb_device,
        0,
        transfer->buffer,
        dev_handle->buffer_size,
        NULL,
        dev_handle,
        0);

    if (transfer->buffer == NULL)
    {
        libusb_free_transfer(libusb_transfer);
        free(transfer);
        return 0;
    }
    return transfer;
}

static void libusb_transport_transfer_free(struct airspyhf_device *dev_handle, struct airspyhf_ll_transfer *transfer)
{
    struct libusb_transfer *_transfer = (struct libusb_transfer *)transfer->transport_data;
    libusb_free_transfer(_transfer);
    free(transfer->buffer);
    free(transfer);
}

void libusb_transport_transfer_set_buffer(struct airspyhf_ll_transfer *transfer, void *buffer)
{
    struct libusb_transfer *_transfer = (struct libusb_transfer *)transfer->transport_data;
    _transfer->buffer = buffer;
    transfer->buffer = buffer;
}

static void LIBUSB_CALL airspyhf_libusb_transfer_callback(struct libusb_transfer *usb_transfer)
{
    struct airspyhf_ll_transfer *transfer = (struct airspyhf_ll_transfer *)usb_transfer->user_data;
    int len_for_callback = 0;
    if (usb_transfer->status == LIBUSB_TRANSFER_COMPLETED && usb_transfer->actual_length == usb_transfer->length)
        len_for_callback = usb_transfer->length;

    if (transfer->callback)
        transfer->callback(transfer, len_for_callback);
}

static int libusb_transport_transfer_prepare(struct airspyhf_device *dev_handle, struct airspyhf_ll_transfer *transfer, ll_transfer_cb_fn callback)
{
    struct libusb_transfer *_transfer = (struct libusb_transfer *)transfer->transport_data;
    transfer->callback = callback;
    _transfer->endpoint = LIBUSB_ENDPOINT_IN | AIRSPYHF_ENDPOINT_IN;
    _transfer->callback = airspyhf_libusb_transfer_callback;
    _transfer->user_data = transfer;
    int error = libusb_submit_transfer(_transfer);

    return error == 0 ? AIRSPYHF_SUCCESS : AIRSPYHF_ERROR;
}

static void libusb_transport_exit(struct airspyhf_transport *transport)
{
    libusb_transport_data_t *transport_data = (libusb_transport_data_t *)transport->transport_data;

    if (transport_data->usb_device != NULL)
    {
        libusb_release_interface(transport_data->usb_device, 0);
        libusb_close(transport_data->usb_device);
        transport_data->usb_device = NULL;
    }
    libusb_exit(transport_data->usb_context);
    transport_data->usb_context = NULL;
}

static int libusb_transport_handle_events(struct airspyhf_device *dev_handle, struct timeval *timeout)
{
    libusb_transport_data_t *transport_data = (libusb_transport_data_t *)dev_handle->transport->transport_data;
    int error = libusb_handle_events_timeout_completed(transport_data->usb_context, timeout, NULL);

    if (error < 0)
    {
        if (error != LIBUSB_ERROR_INTERRUPTED)
            return AIRSPYHF_ERROR;
    }

    return AIRSPYHF_SUCCESS;
}

static int libusb_transport_list_devices(struct airspyhf_transport *transport, uint64_t *serials, int count)
{
    libusb_transport_data_t *transport_data = (libusb_transport_data_t *)transport->transport_data;
    libusb_device_handle *libusb_dev_handle;
    libusb_device **devices = NULL;
    libusb_device *dev;
    struct libusb_device_descriptor device_descriptor;

    int serial_descriptor_index;
    int serial_number_len;
    int output_count;
    int i;
    unsigned char serial_number[AIRSPYHF_SERIAL_SIZE + 1];

    if (serials)
    {
        memset(serials, 0, sizeof(uint64_t) * count);
    }

    if (libusb_get_device_list(transport_data->usb_context, &devices) < 0)
    {
        return AIRSPYHF_ERROR;
    }

    i = 0;
    output_count = 0;
    while ((dev = devices[i++]) != NULL && (!serials || output_count < count))
    {
        libusb_get_device_descriptor(dev, &device_descriptor);

        if ((device_descriptor.idVendor == airspyhf_usb_vid) &&
            (device_descriptor.idProduct == airspyhf_usb_pid))
        {
            serial_descriptor_index = device_descriptor.iSerialNumber;
            if (serial_descriptor_index > 0)
            {
                if (libusb_open(dev, &libusb_dev_handle) != 0)
                {
                    continue;
                }

                serial_number_len = libusb_get_string_descriptor_ascii(libusb_dev_handle,
                                                                       serial_descriptor_index,
                                                                       serial_number,
                                                                       sizeof(serial_number));

                if (serial_number_len == AIRSPYHF_SERIAL_SIZE && !memcmp(str_prefix_serial_airspyhf, serial_number, STR_PREFIX_SERIAL_AIRSPYHF_SIZE))
                {
                    char *start, *end;
                    uint64_t serial;

                    serial_number[AIRSPYHF_SERIAL_SIZE] = 0;
                    start = (char *)(serial_number + STR_PREFIX_SERIAL_AIRSPYHF_SIZE);
                    end = NULL;
                    serial = strtoull(start, &end, 16);
                    if (serial == 0 && start == end)
                    {
                        libusb_close(libusb_dev_handle);
                        continue;
                    }

                    if (serials)
                    {
                        serials[output_count] = serial;
                    }
                    output_count++;
                }

                libusb_close(libusb_dev_handle);
            }
        }
    }

    libusb_free_device_list(devices, 1);
    return output_count;
}

static void libusb_transport_reset(struct airspyhf_device *dev_handle)
{
    libusb_transport_data_t *transport_data = (libusb_transport_data_t *)dev_handle->transport->transport_data;
    libusb_clear_halt(transport_data->usb_device, LIBUSB_ENDPOINT_IN | 1);
}

static void libusb_transport_stop(struct airspyhf_device *dev_handle)
{
#ifndef _WIN32
    libusb_transport_data_t *transport_data = (libusb_transport_data_t *)dev_handle->transport->transport_data;
    libusb_interrupt_event_handler(transport_data->usb_context);
#endif
}

int airspyhf_get_libusb_transport(struct airspyhf_transport *transport)
{
    int libusb_error;

    transport->device_open = libusb_transport_device_open;
    transport->device_reset = libusb_transport_reset;
    transport->device_stop = libusb_transport_stop;
    transport->device_close = libusb_transport_device_close;
    transport->control_transfer = libusb_transport_transfer_control;
    transport->cancel_transfer = libusb_transport_transfer_cancel;
    transport->submit_transfer = libusb_transport_transfer_submit;
    transport->alloc_transfer = libusb_transport_transfer_alloc;
    transport->prepare_transfer = libusb_transport_transfer_prepare;
    transport->free_transfer = libusb_transport_transfer_free;
    transport->exit = libusb_transport_exit;
    transport->handle_events = libusb_transport_handle_events;
    transport->list_devices = libusb_transport_list_devices;
    transport->set_transfer_buffer = libusb_transport_transfer_set_buffer;
    transport->transport_data = malloc(sizeof(libusb_transport_data_t));
    if (transport->transport_data)
    {
        libusb_transport_data_t *data = (libusb_transport_data_t *)(transport->transport_data);
        data->usb_context = 0;
        data->usb_device = 0;
        data->dev_handle = 0;

#ifdef __ANDROID__
        // LibUSB does not support device discovery on android
        libusb_set_option(NULL, LIBUSB_OPTION_NO_DEVICE_DISCOVERY, NULL);
#endif

        libusb_error = libusb_init(&data->usb_context);
        if (libusb_error != 0)
        {
            free(transport->transport_data);
            return AIRSPYHF_ERROR;
        }
        return AIRSPYHF_SUCCESS;
    }
    else
        return AIRSPYHF_ERROR;
}

#endif
