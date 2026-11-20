//---------------------------------------------------------------------------------------
// usb_device_my_bulk.c                                            20210409 CHG 
//
// Modified from NXP's CDC sample from SDK 2.9.2 by Carsten Groen
//---------------------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>

#include "usb_device_config.h"
#include "usb.h"
#include "usb_device.h"

#include "usb_device_class.h"
#include "usb_device_descriptor.h"

#include "usb_device_my_bulk.h"

extern usb_device_endpoint_struct_t g_UsbDeviceMyBulkEndpoints[];


#define USB_MYBULK_ENTER_CRITICAL() \
    OSA_SR_ALLOC();                  \
    OSA_ENTER_CRITICAL()

#define USB_MYBULK_EXIT_CRITICAL() OSA_EXIT_CRITICAL()

// MyBulk device instance 
USB_GLOBAL USB_RAM_ADDRESS_ALIGNMENT(USB_DATA_ALIGN_SIZE) static usb_device_mybulk_struct_t g_myBulkHandle[USB_DEVICE_CONFIG_MYBULK];


//---------------------------------------------------------------------------------------
// Allocates the MyBulk device handle.
//
// This function allocates the MyBulk device handle.
//
// handle The class handle of the MyBulk class.
//  returns A USB error code or kStatus_USB_Success.
//---------------------------------------------------------------------------------------
static usb_status_t USB_DeviceMyBulkAllocateHandle(usb_device_mybulk_struct_t **handle) {
    uint32_t count;
    for (count = 0; count < USB_DEVICE_CONFIG_MYBULK; count++) {
        if (NULL == g_myBulkHandle[count].handle) {
            *handle = &g_myBulkHandle[count];
            return kStatus_USB_Success;
        }
    }
    return kStatus_USB_Busy;
}

//---------------------------------------------------------------------------------------
// Frees the MyBulk device handle.
//
// handle The class handle of the MyBulk class.
// returns A USB error code or kStatus_USB_Success.
//---------------------------------------------------------------------------------------
static usb_status_t USB_DeviceMyBulkFreeHandle(usb_device_mybulk_struct_t *handle) {
    handle->handle        = NULL;
    handle->configStruct  = NULL;
    handle->configuration = 0;
    handle->alternate     = 0;
    return kStatus_USB_Success;
}


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Handle bulk IN events. These functions gets called when a mnessage we sent to the PC has been ACK'ed by the PC
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//---------------------------------------------------------------------------------------
// Common helper function for all IN event.
//
// Responds to a bulk IN endpoint event on a specific endpoint.
//    handle The device handle of the MyBulk device.
//    message The pointer to the message of the endpoint callback.
//    callbackParam The pointer to the parameter of the callback.
//  ep The IN endpoint that should be handled
//  event Hvis event to send to the BulkUSB.c module
//    Return USB error code or kStatus_USB_Success.
//---------------------------------------------------------------------------------------
static usb_status_t handleDeviceResponseBulkIn(usb_device_handle handle, usb_device_endpoint_callback_message_struct_t *message, void *callbackParam, int ep, int event) {
    usb_device_mybulk_struct_t *myBulkHandle;
    usb_status_t status = kStatus_USB_Error;
    int i;

    (void)handle;
   
    myBulkHandle        = (usb_device_mybulk_struct_t *)callbackParam;
    if (NULL == myBulkHandle) {
        return kStatus_USB_InvalidHandle;
    }
   
   // Find our index in bulkIn[] based on the specific EP number we handle in this function
   for (i=0; i<USB_BULK_NUM_ENDPOINTS_IN; i++) {
      if (myBulkHandle->bulkIn[i].ep == ep) {
         break;
      }
   }
   
    myBulkHandle->bulkIn[i].isBusy = 0;
    if ((NULL != myBulkHandle->configStruct) && (NULL != myBulkHandle->configStruct->classCallback)) {
        status = myBulkHandle->configStruct->classCallback((class_handle_t)myBulkHandle, event, message);
    }
    return status;
}


//---------------------------------------------------------------------------------------
// 3 handlers that responds to a bulk IN endpoint event on their specific endpoint, each
// calls a common helper function
//    handle The device handle of the MyBulk device.
//    message The pointer to the message of the endpoint callback.
//    callbackParam The pointer to the parameter of the callback.
//    A USB error code or kStatus_USB_Success.
//---------------------------------------------------------------------------------------
static usb_status_t USB_DeviceResponseBulkIn(usb_device_handle handle, usb_device_endpoint_callback_message_struct_t *message, void *callbackParam) {
   return handleDeviceResponseBulkIn(handle, message, callbackParam, BULK_IN_EP, kUSB_DeviceMyBulkEventResponseTransmitDone);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Handle bulk OUT events. This functions gets called when a message sent by the PC has been received
// We only have 1 single OUT endpoint in this device
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//---------------------------------------------------------------------------------------
// Responds to the bulk out endpoint event
//    handle The device handle of the MyBulk device.
// message The pointer to the message of the endpoint callback.
// callbackParam The pointer to the parameter of the callback.
// return A USB error code or kStatus_USB_Success.
//---------------------------------------------------------------------------------------
static usb_status_t USB_DeviceCommandBulkOut(usb_device_handle handle, usb_device_endpoint_callback_message_struct_t *message, void *callbackParam) {
    usb_device_mybulk_struct_t *myBulkHandle;
    usb_status_t status = kStatus_USB_Error;
    myBulkHandle        = (usb_device_mybulk_struct_t *)callbackParam;

    (void)handle;

    if (NULL == myBulkHandle) {
        return kStatus_USB_InvalidHandle;
    }
    myBulkHandle->bulkOut[0].isBusy = 0U;
    if ((NULL != myBulkHandle->configStruct) && (NULL != myBulkHandle->configStruct->classCallback)) {
        status = myBulkHandle->configStruct->classCallback((class_handle_t)myBulkHandle, kUSB_DeviceMyBulkEventRequestReceiveDone, message);
    }
    return status;
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

 
//---------------------------------------------------------------------------------------
// Initializes the endpoints in MyBulk class.
// myBulkHandle The class handle of the MyBulk class.
// return A USB error code or kStatus_USB_Success.
//---------------------------------------------------------------------------------------
static usb_status_t USB_DeviceMyBulkEndpointsInit(usb_device_mybulk_struct_t *myBulkHandle) {
    usb_device_interface_list_t *interfaceList;
    usb_device_interface_struct_t *interface = NULL;
    usb_status_t error                       = kStatus_USB_Error;
    uint32_t count;
    uint32_t index;
   
   
    if (NULL == myBulkHandle) {
        return error;
    }

    /* return error when configuration is invalid (0 or more than the configuration number) */
    if ((myBulkHandle->configuration == 0U) ||
        (myBulkHandle->configuration > myBulkHandle->configStruct->classInfomation->configurations)) {
        return error;
    }

    interfaceList = &myBulkHandle->configStruct->classInfomation->interfaceList[myBulkHandle->configuration - 1U];

    for (count = 0; count < interfaceList->count; count++) {
        if (USB_DEVICE_CLASS == interfaceList->interfaces[count].classCode) {
            for (index = 0; index < interfaceList->interfaces[count].count; index++) {
                if (interfaceList->interfaces[count].interface[index].alternateSetting == myBulkHandle->alternate) {
                    interface = &interfaceList->interfaces[count].interface[index];
                    break;
                }
            }
            myBulkHandle->interfaceNumber = interfaceList->interfaces[count].interfaceNumber;
            break;
        }
    }
    if (NULL == interface) {
        return error;
    }

   
   //---------------------------------------------------------------------------------------
   // We need to populate the myBulkHandle->bulkIn[] and myBulkHandle->bulkOut[] arrays with
   // the correct callback functions. We look at the specific endpoint and assign the correct
   // callback handler based on that
   //---------------------------------------------------------------------------------------
   int outIndex=0; // Index into myBulkHandle->bulkOut[]
   int inIndex=0;  // Index into myBulkHandle->bulkIn[]

   // Loop thru all the defined endpoints
    for (count = 0; count < interface->endpointList.count; count++) {
        usb_device_endpoint_init_struct_t epInitStruct;
        usb_device_endpoint_callback_struct_t epCallback;
        epInitStruct.zlt             = 0;
        epInitStruct.interval        = interface->endpointList.endpoint[count].interval;
        epInitStruct.endpointAddress = interface->endpointList.endpoint[count].endpointAddress;
        epInitStruct.maxPacketSize   = interface->endpointList.endpoint[count].maxPacketSize;
        epInitStruct.transferType    = interface->endpointList.endpoint[count].transferType;

      // If the endpoint is a IN endpoint, assign callback function etc to the specific myBulkHandle->bulkIn[]
        if ((USB_IN == ((epInitStruct.endpointAddress & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK) >>
                        USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT)) &&
            (USB_ENDPOINT_BULK == epInitStruct.transferType))
        {
            myBulkHandle->bulkIn[inIndex].ep          = (epInitStruct.endpointAddress & USB_DESCRIPTOR_ENDPOINT_ADDRESS_NUMBER_MASK);
            myBulkHandle->bulkIn[inIndex].isBusy      = 0;
            myBulkHandle->bulkIn[inIndex].pipeDataBuffer= (uint8_t *)USB_UNINITIALIZED_VAL_32;
            myBulkHandle->bulkIn[inIndex].pipeStall     = 0;
            myBulkHandle->bulkIn[inIndex].pipeDataLen   = 0;
         
         // Decide whích bulk handler to call depending on endpoint number
         switch (myBulkHandle->bulkIn[inIndex].ep) {
            case BULK_IN_EP: epCallback.callbackFn       = USB_DeviceResponseBulkIn; break;
            default: epCallback.callbackFn=NULL;
         }
            
         inIndex++;
        }
      // If the endpoint is a OUT endpoint, assign callback function etc to the specific myBulkHandle->bulkOut[]
      // (This is pretty simple as we only have 1 single OUT EP in this device :)
        else if ((USB_OUT == ((epInitStruct.endpointAddress & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK) >>
                              USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT)) &&
                 (USB_ENDPOINT_BULK == epInitStruct.transferType))
        {
            myBulkHandle->bulkOut[outIndex].ep              = (epInitStruct.endpointAddress & USB_DESCRIPTOR_ENDPOINT_ADDRESS_NUMBER_MASK);
            myBulkHandle->bulkOut[outIndex].isBusy          = 0;
            myBulkHandle->bulkOut[outIndex].pipeDataBuffer  = (uint8_t *)USB_UNINITIALIZED_VAL_32;
            myBulkHandle->bulkOut[outIndex].pipeStall       = 0;
            myBulkHandle->bulkOut[outIndex].pipeDataLen     = 0;
         
         // Decide which bulk handler to call depending on endpoint number
         switch (myBulkHandle->bulkOut[outIndex].ep) {
            case BULK_OUT_EP: epCallback.callbackFn      = USB_DeviceCommandBulkOut; break;
            default: epCallback.callbackFn=NULL;
         }
         outIndex++;
        } else {
        }
        epCallback.callbackParam = myBulkHandle;
      
      // Register callback handler etc for each endpoint
        error = USB_DeviceInitEndpoint(myBulkHandle->handle, &epInitStruct, &epCallback);
    }
   
    return error;
}

//-----------------------------------------------------------------------------
// De-initializes the endpoints in MyBulk class.
//
// This function de-initializes the endpoints in MyBulk class.
//
// myBulkHandle The class handle of the MyBulk class.
// returns A USB error code or kStatus_USB_Success.
//-----------------------------------------------------------------------------
static usb_status_t USB_DeviceMyBulkEndpointsDeinit(usb_device_mybulk_struct_t *myBulkHandle) {
    usb_status_t status = kStatus_USB_Error;
    uint32_t count;

    if (NULL == myBulkHandle->dataInterfaceHandle) {
        return status;
    }
    for (count = 0; count < myBulkHandle->dataInterfaceHandle->endpointList.count; count++) {
        status = USB_DeviceDeinitEndpoint(myBulkHandle->handle, myBulkHandle->dataInterfaceHandle->endpointList.endpoint[count].endpointAddress);
    }
    myBulkHandle->dataInterfaceHandle = NULL;

    return status;
}

//-----------------------------------------------------------------------------
// Handles the MyBulk class event.
//
// This function responds to various events including the common device events and the class-specific events.
// For class-specific events, it calls the class callback defined in the application to deal with the class-specific event.
//
// handle The class handle of the MyBulk class.
// event The event type.
// param The class handle of the MyBulk class.
// returns:
//    A USB error code or kStatus_USB_Success.
//    kStatus_USB_Success The MyBulk class is de-initialized successfully.
//    kStatus_USB_Error The configure structure of the MyBulk class handle is invalid.
//    kStatus_USB_InvalidHandle The MyBulk device handle or the MyBulk class handle is invalid.
//    kStatus_USB_InvalidParameter The endpoint number of the MyBulk class handle is invalid.
//    Others The error code returned by class callback in application.
//-----------------------------------------------------------------------------
usb_status_t USB_DeviceMyBulkEvent(void *handle, uint32_t event, void *param) {
    usb_device_mybulk_struct_t *myBulkHandle;
    usb_status_t error = kStatus_USB_Error;
    uint16_t interfaceAlternate;
    uint8_t *temp8;
    uint8_t alternate;
    usb_device_class_event_t eventCode = (usb_device_class_event_t)event;
    if ((NULL == param) || (NULL == handle)) {
        return kStatus_USB_InvalidHandle;
    }

    myBulkHandle = (usb_device_mybulk_struct_t *)handle;

   // CHG: We only handle the common class events 
    switch (eventCode) {
        case kUSB_DeviceClassEventDeviceReset:
            // Bus reset, clear the configuration
            myBulkHandle->configuration = 0;
            break;
      
        case kUSB_DeviceClassEventSetConfiguration:
            temp8 = ((uint8_t *)param);
            if (NULL == myBulkHandle->configStruct) {
                break;
            }
            if (*temp8 == myBulkHandle->configuration) {
                break;
            }

            error                       = USB_DeviceMyBulkEndpointsDeinit(myBulkHandle);
            myBulkHandle->configuration = *temp8;
            myBulkHandle->alternate     = 0U;
            error                       = USB_DeviceMyBulkEndpointsInit(myBulkHandle);
            if (kStatus_USB_Success != error) {
            #ifdef DEBUG
               usb_echo("kUSB_DeviceClassEventSetConfiguration, USB_DeviceInitEndpoint fail\r\n");
            #endif
            }
            break;
         
         
        case kUSB_DeviceClassEventSetInterface:
            if (NULL == myBulkHandle->configStruct) {
                break;
            }

            interfaceAlternate = *((uint16_t *)param);
            alternate          = (uint8_t)(interfaceAlternate & 0xFFU);

            if (myBulkHandle->interfaceNumber != ((uint8_t)(interfaceAlternate >> 8U))) {
                break;
            }
            if (alternate == myBulkHandle->alternate) {
                break;
            }
            error                   = USB_DeviceMyBulkEndpointsDeinit(myBulkHandle);
            myBulkHandle->alternate = alternate;
            error                   = USB_DeviceMyBulkEndpointsInit(myBulkHandle);
            if (kStatus_USB_Success != error) {
            #ifdef DEBUG
               usb_echo("kUSB_DeviceClassEventSetInterface, USB_DeviceInitEndpoint fail\r\n");
            #endif
            }
            break;
         
        // CHG Not used
        case kUSB_DeviceClassEventSetEndpointHalt:
            break;

        // CHG Not used
        case kUSB_DeviceClassEventClearEndpointHalt:
            break;
         
         // CHG Not used
         case kUSB_DeviceClassEventClassRequest:
            break;

        default:
            break;
    }
    return error;
}

//-----------------------------------------------------------------------------
// USB MyBulk Class Driver
//
// Initializes the MyBulk class.
//
// This function obtains a USB device handle according to the controller ID, initializes the MyBulk class
// with the class configure parameters and creates the mutex for each pipe.
//
//    controllerId The ID of the controller. The value can be chosen from the kUSB_ControllerKhci0,
//    kUSB_ControllerKhci1, kUSB_ControllerEhci0, or kUSB_ControllerEhci1.
//    config The user configuration structure of type usb_device_class_config_struct_t. The user
// populates the members of this structure and passes the pointer of this structure
// into this function.
//    handle  It is out parameter. The class handle of the MyBulk class.
// returns: 
//    A USB error code or kStatus_USB_Success.
//    kStatus_USB_Success The MyBulk class is initialized successfully.
//    kStatus_USB_Busy No MyBulk device handle available for allocation.
//    kStatus_USB_InvalidHandle The MyBulk device handle allocation failure.
//    kStatus_USB_InvalidParameter The USB device handle allocation failure.
//-----------------------------------------------------------------------------
usb_status_t USB_DeviceMyBulkInit(uint8_t controllerId, usb_device_class_config_struct_t *config, class_handle_t *handle) {
    usb_device_mybulk_struct_t *myBulkHandle;
    usb_status_t error;
   int i;
    error = USB_DeviceMyBulkAllocateHandle(&myBulkHandle);

    if (kStatus_USB_Success != error) {
        return error;
    }

    error = USB_DeviceClassGetDeviceHandle(controllerId, &myBulkHandle->handle);

    if (kStatus_USB_Success != error) {
        return error;
    }

    if (NULL == myBulkHandle->handle) {
        return kStatus_USB_InvalidHandle;
    }
    myBulkHandle->configStruct  = config;
    myBulkHandle->configuration = 0;
    myBulkHandle->alternate     = 0xFF;

   for (i=0; i<USB_BULK_NUM_ENDPOINTS_IN; i++) {
      myBulkHandle->bulkIn[i].mutex = (osa_mutex_handle_t)&myBulkHandle->bulkIn[i].mutexBuffer[0];
      if (KOSA_StatusSuccess != OSA_MutexCreate((myBulkHandle->bulkIn[i].mutex))) {
      #ifdef DEBUG
         usb_echo("mutex create error!");
      #endif
      }
   }
   
   for (i=0; i<USB_BULK_NUM_ENDPOINTS_OUT; i++) {
      myBulkHandle->bulkOut[i].mutex = (osa_mutex_handle_t)&myBulkHandle->bulkOut[i].mutexBuffer[0];
      if (KOSA_StatusSuccess != OSA_MutexCreate((myBulkHandle->bulkOut[i].mutex))) {
         #ifdef DEBUG
            usb_echo("mutex create error!");
         #endif
      }
   }
    *handle = (class_handle_t)myBulkHandle;
    return error;
}

//-----------------------------------------------------------------------------
// Deinitializes the USB MyBulk class.
//
// This function destroys the mutex for each pipe, deinitializes each endpoint of the MyBulk class and frees
// the MyBulk class handle.
// 
// handle The class handle of the MyBulk class.
//    returns:
//    A USB error code or kStatus_USB_Success.
//    kStatus_USB_Success The MyBulk class is de-initialized successfully.
//    kStatus_USB_Error The endpoint deinitialization failure.
//    kStatus_USB_InvalidHandle The MyBulk device handle or the MyBulk class handle is invalid.
//    kStatus_USB_InvalidParameter The endpoint number of the MyBulk class handle is invalid.
//-----------------------------------------------------------------------------
usb_status_t USB_DeviceMyBulkDeinit(class_handle_t handle) {
    usb_device_mybulk_struct_t *myBulkHandle;
    usb_status_t error;
   int i;
    myBulkHandle = (usb_device_mybulk_struct_t *)handle;

    if (NULL == myBulkHandle) {
        return kStatus_USB_InvalidHandle;
    }
   
   for (i=0; i<USB_BULK_NUM_ENDPOINTS_IN; i++) {
      if (KOSA_StatusSuccess != OSA_MutexDestroy((myBulkHandle->bulkIn[i].mutex))) {
      #ifdef DEBUG
         usb_echo("mutex destroy error!");
      #endif
      }
   }
   
   for (i=0; i<USB_BULK_NUM_ENDPOINTS_OUT; i++) {
      if (KOSA_StatusSuccess != OSA_MutexDestroy((myBulkHandle->bulkOut[i].mutex))) {
         #ifdef DEBUG
            usb_echo("mutex destroy error!");
         #endif
      }
   }
    error = USB_DeviceMyBulkEndpointsDeinit(myBulkHandle);
    (void)USB_DeviceMyBulkFreeHandle(myBulkHandle);
    return error;
}

//-----------------------------------------------------------------------------
// Primes the endpoint to send packet to host.
//
// This function checks whether the endpoint is sending packet, then it primes the endpoint
// with the buffer address and the buffer length if the pipe is not busy. Otherwise, it ignores this transfer by
// returning an error code.
// 
// handle The class handle of the MyBulk class.
// ep The endpoint number of the transfer.
// buffer The pointer to the buffer to be transferred.
// length The length of the buffer to be transferred.
// returns:
//    A USB error code or kStatus_USB_Success.
//    kStatus_USB_Success Prime to send packet successfully.
//    kStatus_USB_Busy The endpoint is busy in transferring.
//    //    kStatus_USB_InvalidHandle The MyBulk device handle or the MyBulk class handle is invalid.
//    kStatus_USB_ControllerNotFound The controller interface is invalid.
//
// The function can only be called in the same context.
//-----------------------------------------------------------------------------
usb_status_t USB_DeviceMyBulkSend(class_handle_t handle, uint8_t ep, uint8_t *buffer, uint32_t length) {
    usb_device_mybulk_struct_t *myBulkHandle;
    usb_status_t status                   = kStatus_USB_Error;
    usb_device_mybulk_pipe_t *myBulkPipe = NULL;
   int i;
   
    if (NULL == handle) {
        return kStatus_USB_InvalidHandle;
    }
    myBulkHandle = (usb_device_mybulk_struct_t *)handle;

   
   // Find our index in bulkIn[] base on EP number we handle in this function
   for (i=0; i<USB_BULK_NUM_ENDPOINTS_IN; i++) {
      if (myBulkHandle->bulkIn[i].ep == ep) {
         break;
      }
   }
   
   // Create a pointer to this specific pipe
   myBulkPipe = &(myBulkHandle->bulkIn[i]);
   
    if (NULL != myBulkPipe) {
        if (1U == myBulkPipe->isBusy) {
            return kStatus_USB_Busy;
        }
        myBulkPipe->isBusy = 1U;

        if (0u != myBulkPipe->pipeStall) {
            myBulkPipe->pipeDataBuffer = buffer;
            myBulkPipe->pipeDataLen    = length;
            return kStatus_USB_Success;
        }

        status = USB_DeviceSendRequest(myBulkHandle->handle, ep, buffer, length);
        if (kStatus_USB_Success != status) {
            myBulkPipe->isBusy = 0U;
        }
    }
    return status;
}

//-----------------------------------------------------------------------------
// Primes the endpoint to receive packet from host.
//
// This function checks whether the endpoint is receiving packet, then it primes the endpoint
// with the buffer address and the buffer length if the pipe is not busy. Otherwise, it ignores this transfer by
// returning an error code.
//
//    handle The class handle of the MyBulk class.
//    ep The endpoint number of the transfer.
//    buffer The pointer to the buffer to be transferred.
//    length The length of the buffer to be transferred.
//    returns:
//    A USB error code or kStatus_USB_Success.
//    kStatus_USB_Success Prime to receive packet successfully.
//    kStatus_USB_Busy The endpoint is busy in transferring.
//    kStatus_USB_InvalidHandle The MyBulk device handle or the MyBulk class handle is invalid.
//    kStatus_USB_ControllerNotFound The controller interface is invalid.
//
// The function can only be called in the same context.
//-----------------------------------------------------------------------------
usb_status_t USB_DeviceMyBulkRecv(class_handle_t handle, uint8_t ep, uint8_t *buffer, uint32_t length) {
    usb_device_mybulk_struct_t *myBulkHandle;
    usb_status_t status;
   int i=0;
    if (NULL == handle) {
        return kStatus_USB_InvalidHandle;
    }
    myBulkHandle = (usb_device_mybulk_struct_t *)handle;

   // Find our index in bulkOut[] base on EP number we handle in this function
   for (i=0; i<USB_BULK_NUM_ENDPOINTS_OUT; i++) {
      if (myBulkHandle->bulkOut[i].ep == ep) {
         break;
      }
   }
   
   
    if (1U == myBulkHandle->bulkOut[i].isBusy) {
        return kStatus_USB_Busy;
    }
    myBulkHandle->bulkOut[i].isBusy = 1U;

    if (0U != myBulkHandle->bulkOut[i].pipeStall) {
        myBulkHandle->bulkOut[i].pipeDataBuffer = buffer;
        myBulkHandle->bulkOut[i].pipeDataLen    = length;
        return kStatus_USB_Success;
    }

    status = USB_DeviceRecvRequest(myBulkHandle->handle, ep, buffer, length);
    if (kStatus_USB_Success != status) {
        myBulkHandle->bulkOut[i].isBusy = 0U;
    }
    return status;
}

