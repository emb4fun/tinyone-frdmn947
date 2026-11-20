//---------------------------------------------------------------------------------------
// usb_device_my_bulk.h                                            20210409 CHG 
//
// Modified from NXP's CDC sample from SDK 2.9.2 by Carsten Groen
//---------------------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------

#ifndef _USB_DEVICE_MYBULK_H_
#define _USB_DEVICE_MYBULK_H_

#include "usb_device_descriptor.h"


//-----------------------------------------------------------------------------
// Definition of MyBulk class event
//-----------------------------------------------------------------------------
typedef enum _usb_device_mybulk_event
{
   kUSB_DeviceMyBulkEventSendResponse = 0x01,     /*!< This event indicates the bulk send transfer is complete or cancelled etc. */
   kUSB_DeviceMyBulkEventRecvResponse,            /*!< This event indicates the bulk receive transfer is complete or cancelled etc.. */
   
   kUSB_DeviceMyBulkEventRequestReceiveDone,    // Event indicates that we have received a message on the "Request" OUT EP
   kUSB_DeviceMyBulkEventResponseTransmitDone,     // Event indicates that we have sent a message on the "Response" IN EP
   
} usb_device_mybulk_event_t;

//-----------------------------------------------------------------------------
// Definition of parameters for MyBulk request
//-----------------------------------------------------------------------------
typedef struct _usb_device_mybulk_request_param_struct
{
    uint8_t **buffer;        /*!< The pointer to the address of the buffer for MyBulk class request. */
    uint32_t *length;        /*!< The pointer to the length of the buffer for MyBulk class request. */
    uint16_t interfaceIndex; /*!< The interface index of the setup packet. */
    uint16_t setupValue;     /*!< The wValue field of the setup packet. */
    uint8_t isSetup;         /*!< The flag indicates if it is a setup packet, 1: yes, 0: no. */
} usb_device_mybulk_request_param_struct_t;

//-----------------------------------------------------------------------------
// Definition of pipe structure
//-----------------------------------------------------------------------------
typedef struct _usb_device_mybulk_pipe
{
    osa_mutex_handle_t mutex; /*!< The mutex of the pipe. */
    uint32_t mutexBuffer[(OSA_MUTEX_HANDLE_SIZE + 3)/4];
    uint8_t *pipeDataBuffer;      /*!< pipe data buffer backup when stall */
    uint32_t pipeDataLen;         /*!< pipe data length backup when stall  */
    uint8_t pipeStall;            /*!< pipe is stall  */
    uint8_t ep;                 /*!< The endpoint number of the pipe. */
    uint8_t isBusy;             /*!< 1: The pipe is transferring packet, 0: The pipe is idle. */
} usb_device_mybulk_pipe_t;

//-----------------------------------------------------------------------------
// Definition of structure for MyBulk device
//-----------------------------------------------------------------------------
typedef struct _usb_device_mybulk_struct {
    usb_device_handle handle;                           /*!< The handle of the USB device. */
    usb_device_class_config_struct_t *configStruct;     /*!< The class configure structure. */
    usb_device_interface_struct_t *dataInterfaceHandle; /*!< The MyBulk data interface handle. */
   
    usb_device_mybulk_pipe_t bulkIn[USB_BULK_NUM_ENDPOINTS_IN];             /*!< The bulk in pipe for sending packet to host. */

    usb_device_mybulk_pipe_t bulkOut[USB_BULK_NUM_ENDPOINTS_OUT];             /*!< The bulk out pipe for receiving packet from host. */

    uint8_t configuration;                            /*!< The current configuration value. */
    uint8_t interfaceNumber;                    /*!< The current interface number. */
    uint8_t alternate;                                /*!< The alternate setting value of the interface. */
} usb_device_mybulk_struct_t;


#if defined(__cplusplus)
extern "C" {
#endif

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
extern usb_status_t USB_DeviceMyBulkInit(uint8_t controllerId,
                                         usb_device_class_config_struct_t *config,
                                         class_handle_t *handle);

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
extern usb_status_t USB_DeviceMyBulkDeinit(class_handle_t handle);


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
extern usb_status_t USB_DeviceMyBulkEvent(void *handle, uint32_t event, void *param);

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
extern usb_status_t USB_DeviceMyBulkSend(class_handle_t handle, uint8_t ep, uint8_t *buffer, uint32_t length);

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
extern usb_status_t USB_DeviceMyBulkRecv(class_handle_t handle, uint8_t ep, uint8_t *buffer, uint32_t length);

/*! @}*/

#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* _USB_DEVICE_MYBULK_H_ */
