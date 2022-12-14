/******************************************************************************
* File Name:   tcp_server.c
*
* Description: This file contains declaration of task and functions related to
* TCP server operation.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2022, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

/* TCP server task header file. */
#include "tcp_server.h"

/* Header file includes */
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

/* Ethernet connection manager header files */
#include "cy_ecm.h"
#include "cy_ecm_error.h"

/* Cypress secure socket header file */
#include "cy_secure_sockets.h"

/* Network connectivity utility header file. */
#include "cy_nw_helper.h"

/* FreeRTOS header file */
#include <FreeRTOS.h>
#include <task.h>

/* Standard C header files */
#include <string.h>
#include <inttypes.h>

/*******************************************************************************
* Macros
********************************************************************************/

/* Maximum number of connection retries to the ethernet network */
#define MAX_ETH_RETRY_COUNT                       (3u)

/* TCP keep alive related macros. */
#define TCP_KEEP_ALIVE_IDLE_TIME_MS               (10000u)
#define TCP_KEEP_ALIVE_INTERVAL_MS                (1000u)
#define TCP_KEEP_ALIVE_RETRY_COUNT                (2u)

/* Length of the LED ON/OFF command issued from the TCP server. */
#define TCP_LED_CMD_LEN                           (1)

/* LED ON and LED OFF commands. */
#define LED_ON_CMD                                '1'
#define LED_OFF_CMD                               '0'

/* Interrupt priority of the user button. */
#define USER_BTN_INTR_PRIORITY                    (5)

/* Debounce delay for user button. */
#define DEBOUNCE_DELAY_MS                         (50)

/* Length of Buffer to store IP address */
#define IP_ADDR_BUFFER_SIZE                       (50)

/*******************************************************************************
* Function Prototypes
********************************************************************************/

static cy_rslt_t create_tcp_server_socket(void);
static cy_rslt_t tcp_connection_handler(cy_socket_t socket_handle, void *arg);
static cy_rslt_t tcp_receive_msg_handler(cy_socket_t socket_handle, void *arg);
static cy_rslt_t tcp_disconnection_handler(cy_socket_t socket_handle, void *arg);
static void isr_button_press( void *callback_arg, cyhal_gpio_event_t event);

/* Establish ethernet connection to the network */
static cy_rslt_t connect_to_ethernet(void);

/*******************************************************************************
* Global Variables
********************************************************************************/
/* Secure socket variables. */
cy_socket_sockaddr_t tcp_server_addr, peer_addr;
cy_socket_t server_handle, client_handle;

/* Size of the peer socket address. */
uint32_t peer_addr_len;

/* Flags to track the LED state. */
bool led_state = CYBSP_LED_STATE_OFF;

/* TCP server task handle. */
extern TaskHandle_t server_task_handle;

/* Ethernet connection manager handle. */
cy_ecm_t ecm_handle = NULL;

/* Flag variable to check if TCP client is connected. */
bool client_connected;

cyhal_gpio_callback_data_t cb_data =
{
.callback = isr_button_press,
.callback_arg = NULL
};

/*******************************************************************************
 * Function Name: tcp_server_task
 *******************************************************************************
 * Summary:
 *  Task used to establish a connection to a TCP client.
 *
 * Parameters:
 *  void *args : Task parameter defined during task creation (unused)
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void tcp_server_task(void *arg)
{
    cy_rslt_t result;

    /* Variable to store number of bytes sent over TCP socket. */
    uint32_t bytes_sent = 0;

    /* Variable to receive LED ON/OFF command from the user button ISR. */
    uint32_t led_state_cmd = LED_OFF_CMD;

    /* Initialize the user button (CYBSP_USER_BTN) and register interrupt on falling edge. */
    cyhal_gpio_init(CYBSP_USER_BTN, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLUP, CYBSP_BTN_OFF);
    cyhal_gpio_register_callback(CYBSP_USER_BTN, &cb_data);
    cyhal_gpio_enable_event(CYBSP_USER_BTN, CYHAL_GPIO_IRQ_FALL, USER_BTN_INTR_PRIORITY, true);

    /* Connect to ethernet network. */
    result = connect_to_ethernet();
    if(result!= CY_RSLT_SUCCESS )
    {
        printf("\n Failed to connect to the ethernet network! Error code: 0x%08"PRIx32"\n", (uint32_t)result);
        CY_ASSERT(0);
    }

    /* Initialize secure socket library. */
    result = cy_socket_init();
    if (result != CY_RSLT_SUCCESS)
    {
        printf("Secure Socket initialization failed! Error code: 0x%08"PRIx32"\n", (uint32_t)result);
        CY_ASSERT(0);
    }
    printf("Secure Socket initialized.\n");

    /* Create TCP server socket. */
    result = create_tcp_server_socket();
    if (result != CY_RSLT_SUCCESS)
    {
        printf("Failed to create socket! Error code: 0x%08"PRIx32"\n", (uint32_t)result);
        CY_ASSERT(0);
    }

    /* Start listening on the TCP server socket. */
    result = cy_socket_listen(server_handle, TCP_SERVER_MAX_PENDING_CONNECTIONS);
    if (result != CY_RSLT_SUCCESS)
    {
        cy_socket_delete(server_handle);
        printf("cy_socket_listen returned error. Error code: 0x%08"PRIx32"\n", (uint32_t)result);
        CY_ASSERT(0);
    }
    else
    {
        printf("===============================================================\n");
        printf("Listening for incoming TCP client connection on Port: %d\n",
                tcp_server_addr.port);
    }

    while(true)
    {
        /* Wait till user button is pressed to send LED ON/OFF command to TCP client. */
        xTaskNotifyWait(0, 0, &led_state_cmd, portMAX_DELAY);

        /* Disable the GPIO signal falling edge detection until the command is
         * sent to the TCP client.
         */
        cyhal_gpio_enable_event(CYBSP_USER_BTN, CYHAL_GPIO_IRQ_FALL, USER_BTN_INTR_PRIORITY, false);

        /* Wait till the debounce period of the user button. */
        vTaskDelay(DEBOUNCE_DELAY_MS/portTICK_PERIOD_MS);

        if(!cyhal_gpio_read(CYBSP_USER_BTN))
        {
            /* Send LED ON/OFF command to TCP client if there is an active
             * TCP client connection.
             */
            if(client_connected)
            {
                /* Send the command to TCP client. */
                result = cy_socket_send(client_handle, &led_state_cmd, TCP_LED_CMD_LEN,
                               CY_SOCKET_FLAGS_NONE, &bytes_sent);
                if(result == CY_RSLT_SUCCESS )
                {
                    if(led_state_cmd == LED_ON_CMD)
                    {
                        printf("LED ON command sent to TCP client\n");
                    }
                    else
                    {
                        printf("LED OFF command sent to TCP client\n");
                    }
                }
                else
                {
                    printf("Failed to send command to client. Error code: 0x%08"PRIx32"\n", (uint32_t)result);
                    if(result == CY_RSLT_MODULE_SECURE_SOCKETS_CLOSED)
                    {
                        /* Disconnect the socket. */
                        cy_socket_disconnect(client_handle, 0);
                        /* Delete the socket. */
                        cy_socket_delete(client_handle);
                    }
                }
            }
        }

        /* Enable the GPIO signal falling edge detection. */
        cyhal_gpio_enable_event(CYBSP_USER_BTN, CYHAL_GPIO_IRQ_FALL, USER_BTN_INTR_PRIORITY, true);
    }
 }

/*******************************************************************************
 * Function Name: connect_to_ethernet
 *******************************************************************************
 * Summary:
 *  Connects to ethernet, retries up to a
 *  configured number of times until the connection succeeds.
 *
 *******************************************************************************/
cy_rslt_t connect_to_ethernet(void)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    uint8_t retry_count = 0;

    /* Variables used by ethernet connection manager.*/
    cy_ecm_phy_config_t ecm_phy_config;
    cy_ecm_ip_address_t ip_addr;

    #if ENABLE_STATIC_IP_ADDRESS
    cy_ecm_ip_setting_t static_ip_addr;

    static_ip_addr.ip_address.version = CY_ECM_IP_VER_V4;
    static_ip_addr.ip_address.ip.v4 = TCP_STATIC_IP_ADDR;
    static_ip_addr.gateway.version = CY_ECM_IP_VER_V4;
    static_ip_addr.gateway.ip.v4 = TCP_STATIC_GATEWAY;
    static_ip_addr.netmask.version = CY_ECM_IP_VER_V4;
    static_ip_addr.netmask.ip.v4 = TCP_NETMASK;
    #endif

    ecm_phy_config.interface_speed_type = CY_ECM_SPEED_TYPE_RGMII;
    ecm_phy_config.mode = CY_ECM_DUPLEX_AUTO;
    ecm_phy_config.phy_speed = CY_ECM_PHY_SPEED_AUTO;

    /* Initialize ethernet connection manager. */
    result = cy_ecm_init();
    if (result != CY_RSLT_SUCCESS)
    {
        printf("Ethernet connection manager initialization failed! Error code: 0x%08"PRIx32"\n", (uint32_t)result);
        CY_ASSERT(0);
    }
    else
    {
        printf("Ethernet connection manager initialized.\n");
    }

    /* To change the MAC address,enter the desired MAC as the second parameter 
    in cy_ecm_ethif_init() instead of NULL. Default MAC address(00-03-19-45-00-00) 
    is used when NULL is passed. */
    result =  cy_ecm_ethif_init(CY_ECM_INTERFACE_ETH1, NULL, &ecm_phy_config, &ecm_handle);
    if (result != CY_RSLT_SUCCESS)
    {
        printf("Ethernet interface initialization failed! Error code: 0x%08"PRIx32"\n", (uint32_t)result);
        CY_ASSERT(0);
    }

    /* Establish a connection to the ethernet network */
    while(1)
    {
        #if ENABLE_STATIC_IP_ADDRESS
        /* Connect to the ethernet network with the assigned static IP address */
        result = cy_ecm_connect(ecm_handle, &static_ip_addr, &ip_addr);
        #else
        /* Connect to the ethernet network with the dynamically allocated IP address by DHCP */
        result = cy_ecm_connect(ecm_handle, NULL, &ip_addr);
        #endif
        if(result != CY_RSLT_SUCCESS)
        {
            retry_count++;
            if (retry_count >= MAX_ETH_RETRY_COUNT)
            {
                printf("Exceeded max ethernet connection attempts\n");
                return result;
            }
            printf("Connection to ethernet network failed. Retrying...\n");
            continue;
        }
        else
        {
            printf("Successfully connected to ethernet network.\n");
            printf("IP Address Assigned: %d.%d.%d.%d\n", (uint8)ip_addr.ip.v4,(uint8)(ip_addr.ip.v4 >> 8), (uint8)(ip_addr.ip.v4 >> 16),
                    (uint8)(ip_addr.ip.v4 >> 24));

            /* IP address and TCP port number of the TCP server */
            tcp_server_addr.ip_address.ip.v4 = ip_addr.ip.v4;
            tcp_server_addr.ip_address.version = CY_SOCKET_IP_VER_V4;
            tcp_server_addr.port = TCP_SERVER_PORT;

            break;
        }
    }
    return result;
}


/*******************************************************************************
 * Function Name: create_tcp_server_socket
 *******************************************************************************
 * Summary:
 *  Function to create a socket and set the socket options
 *
 *******************************************************************************/
static cy_rslt_t create_tcp_server_socket(void)
{
    cy_rslt_t result;
    /* TCP socket receive timeout period. */
    uint32_t tcp_recv_timeout = TCP_SERVER_RECV_TIMEOUT_MS;

    /* Variables used to set socket options. */
    cy_socket_opt_callback_t tcp_receive_option;
    cy_socket_opt_callback_t tcp_connection_option;
    cy_socket_opt_callback_t tcp_disconnection_option;

    /* Create a TCP socket */
    result = cy_socket_create(CY_SOCKET_DOMAIN_AF_INET, CY_SOCKET_TYPE_STREAM,
                              CY_SOCKET_IPPROTO_TCP, &server_handle);
    if(result != CY_RSLT_SUCCESS)
    {
        printf("Failed to create socket! Error code: 0x%08"PRIx32"\n", (uint32_t)result);
        return result;
    }

    /* Set the TCP socket receive timeout period. */
    result = cy_socket_setsockopt(server_handle, CY_SOCKET_SOL_SOCKET,
                                 CY_SOCKET_SO_RCVTIMEO, &tcp_recv_timeout,
                                 sizeof(tcp_recv_timeout));
    if(result != CY_RSLT_SUCCESS)
    {
        printf("Set socket option: CY_SOCKET_SO_RCVTIMEO failed\n");
        return result;
    }

    /* Register the callback function to handle connection request from a TCP client. */
    tcp_connection_option.callback = tcp_connection_handler;
    tcp_connection_option.arg = NULL;

    result = cy_socket_setsockopt(server_handle, CY_SOCKET_SOL_SOCKET,
                                  CY_SOCKET_SO_CONNECT_REQUEST_CALLBACK,
                                  &tcp_connection_option, sizeof(cy_socket_opt_callback_t));
    if(result != CY_RSLT_SUCCESS)
    {
        printf("Set socket option: CY_SOCKET_SO_CONNECT_REQUEST_CALLBACK failed\n");
        return result;
    }

    /* Register the callback function to handle messages received from a TCP client. */
    tcp_receive_option.callback = tcp_receive_msg_handler;
    tcp_receive_option.arg = NULL;

    result = cy_socket_setsockopt(server_handle, CY_SOCKET_SOL_SOCKET,
                                  CY_SOCKET_SO_RECEIVE_CALLBACK,
                                  &tcp_receive_option, sizeof(cy_socket_opt_callback_t));
    if(result != CY_RSLT_SUCCESS)
    {
        printf("Set socket option: CY_SOCKET_SO_RECEIVE_CALLBACK failed\n");
        return result;
    }

    /* Register the callback function to handle disconnection. */
    tcp_disconnection_option.callback = tcp_disconnection_handler;
    tcp_disconnection_option.arg = NULL;

    result = cy_socket_setsockopt(server_handle, CY_SOCKET_SOL_SOCKET,
                                  CY_SOCKET_SO_DISCONNECT_CALLBACK,
                                  &tcp_disconnection_option, sizeof(cy_socket_opt_callback_t));
    if(result != CY_RSLT_SUCCESS)
    {
        printf("Set socket option: CY_SOCKET_SO_DISCONNECT_CALLBACK failed\n");
        return result;
    }

    /* Bind the TCP socket created to Server IP address and to TCP port. */
    result = cy_socket_bind(server_handle, &tcp_server_addr, sizeof(tcp_server_addr));
    if(result != CY_RSLT_SUCCESS)
    {
        printf("Failed to bind to socket! Error code: 0x%08"PRIx32"\n", (uint32_t)result);
    }
    
    return result;
}

 /*******************************************************************************
 * Function Name: tcp_connection_handler
 *******************************************************************************
 * Summary:
 *  Callback function to handle incoming TCP client connection.
 *
 * Parameters:
 * cy_socket_t socket_handle: Connection handle for the TCP server socket
 *  void *args : Parameter passed on to the function (unused)
 *
 * Return:
 *  cy_result result: Result of the operation
 *
 *******************************************************************************/
static cy_rslt_t tcp_connection_handler(cy_socket_t socket_handle, void *arg)
{
    cy_rslt_t result;
    char ip_addr_ptr[IP_ADDR_BUFFER_SIZE];

    /* TCP keep alive parameters. */
    int keep_alive = 1;
    uint32_t keep_alive_interval = TCP_KEEP_ALIVE_INTERVAL_MS;
    uint32_t keep_alive_count    = TCP_KEEP_ALIVE_RETRY_COUNT;
    uint32_t keep_alive_idle_time = TCP_KEEP_ALIVE_IDLE_TIME_MS;

    /* Accept new incoming connection from a TCP client.*/
    result = cy_socket_accept(socket_handle, &peer_addr, &peer_addr_len,
                              &client_handle);
    if(result == CY_RSLT_SUCCESS)
    {
        printf("Incoming TCP connection accepted: ");

        cy_nw_ntoa((cy_nw_ip_address_t *)&peer_addr.ip_address, ip_addr_ptr);
        printf("IP Address : %s\n\n", ip_addr_ptr);
        printf("Press the user button to send LED ON/OFF command to the TCP client\n");

        /* Set the TCP keep alive interval. */
        result = cy_socket_setsockopt(client_handle, CY_SOCKET_SOL_TCP,
                                      CY_SOCKET_SO_TCP_KEEPALIVE_INTERVAL,
                                      &keep_alive_interval, sizeof(keep_alive_interval));
        if(result != CY_RSLT_SUCCESS)
        {
            printf("Set socket option: CY_SOCKET_SO_TCP_KEEPALIVE_INTERVAL failed\n");
            return result;
        }

        /* Set the retry count for TCP keep alive packet. */
        result = cy_socket_setsockopt(client_handle, CY_SOCKET_SOL_TCP,
                                      CY_SOCKET_SO_TCP_KEEPALIVE_COUNT,
                                      &keep_alive_count, sizeof(keep_alive_count));
        if(result != CY_RSLT_SUCCESS)
        {
            printf("Set socket option: CY_SOCKET_SO_TCP_KEEPALIVE_COUNT failed\n");
            return result;
        }

        /* Set the network idle time before sending the TCP keep alive packet. */
        result = cy_socket_setsockopt(client_handle, CY_SOCKET_SOL_TCP,
                                      CY_SOCKET_SO_TCP_KEEPALIVE_IDLE_TIME,
                                      &keep_alive_idle_time, sizeof(keep_alive_idle_time));
        if(result != CY_RSLT_SUCCESS)
        {
            printf("Set socket option: CY_SOCKET_SO_TCP_KEEPALIVE_IDLE_TIME failed\n");
            return result;
        }

        /* Enable TCP keep alive. */
        result = cy_socket_setsockopt(client_handle, CY_SOCKET_SOL_SOCKET,
                                          CY_SOCKET_SO_TCP_KEEPALIVE_ENABLE,
                                              &keep_alive, sizeof(keep_alive));
        if(result != CY_RSLT_SUCCESS)
        {
            printf("Set socket option: CY_SOCKET_SO_TCP_KEEPALIVE_ENABLE failed\n");
            return result;
        }

        /* Set the client connection flag as true. */
        client_connected = true;
    }
    else
    {
        printf("Failed to accept incoming client connection. Error code: 0x%08"PRIx32"\n", (uint32_t)result);
        printf("===============================================================\r\n");
        printf("Listening for incoming TCP client connection on Port: %d\n",
                tcp_server_addr.port);
    }

    return result;
}

 /*******************************************************************************
 * Function Name: tcp_receive_msg_handler
 *******************************************************************************
 * Summary:
 *  Callback function to handle incoming TCP client messages.
 *
 * Parameters:
 * cy_socket_t socket_handle: Connection handle for the TCP client socket
 *  void *args : Parameter passed on to the function (unused)
 *
 * Return:
 *  cy_result result: Result of the operation
 *
 *******************************************************************************/
static cy_rslt_t tcp_receive_msg_handler(cy_socket_t socket_handle, void *arg)
{
    char message_buffer[MAX_TCP_RECV_BUFFER_SIZE];
    cy_rslt_t result;

    /* Variable to store number of bytes received from TCP client. */
    uint32_t bytes_received = 0;
    result = cy_socket_recv(socket_handle, message_buffer, MAX_TCP_RECV_BUFFER_SIZE,
                            CY_SOCKET_FLAGS_NONE, &bytes_received);

    if(result == CY_RSLT_SUCCESS)
    {
        /* Terminate the received string with '\0'. */
        message_buffer[bytes_received] = '\0';
        printf("\r\nAcknowledgement from TCP Client: %s\n", message_buffer);

        /* Set the LED state based on the acknowledgement received from the TCP client. */
        if(strcmp(message_buffer, "LED ON ACK") == 0)
        {
            led_state = CYBSP_LED_STATE_ON;
        }
        else
        {
            led_state = CYBSP_LED_STATE_OFF;
        }
    }
    else
    {
        printf("Failed to receive acknowledgement from the TCP client. Error: 0x%08"PRIx32"\n",
              (uint32_t)result);
        if(result == CY_RSLT_MODULE_SECURE_SOCKETS_CLOSED)
        {
            /* Disconnect the socket. */
            cy_socket_disconnect(socket_handle, 0);
            /* Delete the socket. */
            cy_socket_delete(socket_handle);
        }
    }

    printf("===============================================================\n");
    printf("Press the user button to send LED ON/OFF command to the TCP client\n");

    return result;
}

 /*******************************************************************************
 * Function Name: tcp_disconnection_handler
 *******************************************************************************
 * Summary:
 *  Callback function to handle TCP client disconnection event.
 *
 * Parameters:
 * cy_socket_t socket_handle: Connection handle for the TCP client socket
 *  void *args : Parameter passed on to the function (unused)
 *
 * Return:
 *  cy_result result: Result of the operation
 *
 *******************************************************************************/
static cy_rslt_t tcp_disconnection_handler(cy_socket_t socket_handle, void *arg)
{
    cy_rslt_t result;

    /* Disconnect the TCP client. */
    result = cy_socket_disconnect(socket_handle, 0);
    /* Delete the socket. */
    cy_socket_delete(socket_handle);

    /* Set the client connection flag as false. */
    client_connected = false;
    printf("TCP Client disconnected! Please reconnect the TCP Client\n");
    printf("===============================================================\n");
    printf("Listening for incoming TCP client connection on Port:%d\n",
            tcp_server_addr.port);

    /* Set the LED state to OFF when the TCP client disconnects. */
    led_state = CYBSP_LED_STATE_OFF;

    return result;
}

/*******************************************************************************
 * Function Name: isr_button_press
 *******************************************************************************
 *
 * Summary:
 *  GPIO interrupt service routine. This function detects button presses and
 *  sets the command to be sent to TCP client.
 *
 * Parameters:
 *  void *callback_arg : pointer to the variable passed to the ISR
 *  cyhal_gpio_event_t event : GPIO event type
 *
 * Return:
 *  None
 *
 *******************************************************************************/
static void isr_button_press( void *callback_arg, cyhal_gpio_event_t event)
{ 
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    /* Variable to hold the LED ON/OFF command to be sent to the TCP client. */
    uint32_t led_state_cmd;

    /* Set the command to be sent to TCP client. */
    if(led_state == CYBSP_LED_STATE_ON)
    {
        led_state_cmd = LED_OFF_CMD;
    }
    else
    {
        led_state_cmd = LED_ON_CMD;
    }

    /* Set the flag to send command to TCP client. */
    xTaskNotifyFromISR(server_task_handle, led_state_cmd,
                      eSetValueWithoutOverwrite, &xHigherPriorityTaskWoken);

    /* Force a context switch if xHigherPriorityTaskWoken is now set to pdTRUE. */
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* [] END OF FILE */
