/* Copyright (c) Nordic Semiconductor ASA
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 *   1. Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * 
 *   2. Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 * 
 *   3. Neither the name of Nordic Semiconductor ASA nor the names of other
 *   contributors to this software may be used to endorse or promote products
 *   derived from this software without specific prior written permission.
 * 
 *   4. This software must only be used in a processor manufactured by Nordic
 *   Semiconductor ASA, or in a processor manufactured by a third party that
 *   is used in combination with a processor manufactured by Nordic Semiconductor.
 * 
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "hal_radio.h"
#include "hal_timer.h"
#include "hal_clock.h"
#include "hal_serial.h"
#include "hal_twi.h"
#include "nrf_gpio.h"

#include "nrf.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>


#define LED1 17
#define LED2 18
/* #define DBG_RADIO_ACTIVE_ENABLE      */

/* #define DBG_BEACON_START_PIN    (17) */
/* #define DBG_HFCLK_ENABLED_PIN   (18) */
/* #define DBG_HFCLK_DISABLED_PIN  (26) */
/* #define DBG_PKT_SENT_PIN        ( 9) */
/* #define DBG_CALCULATE_BEGIN_PIN (15) */
/* #define DBG_CALCULATE_END_PIN   (15) */
/* #define DBG_RADIO_ACTIVE_PIN    (24) */
/* #define DBG_WFE_BEGIN_PIN       (25) */
/* #define DBG_WFE_END_PIN         (25) */


/* #define DBG_BEACON_START                           \
{                                                     \
    NRF_GPIO->OUTSET = (1 << DBG_BEACON_START_PIN);   \
    NRF_GPIO->DIRSET = (1 << DBG_BEACON_START_PIN);   \
    NRF_GPIO->DIRCLR = (1 << DBG_BEACON_START_PIN);   \
    NRF_GPIO->OUTCLR = (1 << DBG_BEACON_START_PIN);   \
} */
/* #define DBG_HFCLK_ENABLED                          \
{                                                     \
    NRF_GPIO->OUTSET = (1 << DBG_HFCLK_ENABLED_PIN);  \
    NRF_GPIO->DIRSET = (1 << DBG_HFCLK_ENABLED_PIN);  \
    NRF_GPIO->DIRCLR = (1 << DBG_HFCLK_ENABLED_PIN);  \
    NRF_GPIO->OUTCLR = (1 << DBG_HFCLK_ENABLED_PIN);  \
} */
/* #define DBG_HFCLK_DISABLED                         \
{                                                     \
    NRF_GPIO->OUTSET = (1 << DBG_HFCLK_DISABLED_PIN); \
    NRF_GPIO->DIRSET = (1 << DBG_HFCLK_DISABLED_PIN); \
    NRF_GPIO->DIRCLR = (1 << DBG_HFCLK_DISABLED_PIN); \
    NRF_GPIO->OUTCLR = (1 << DBG_HFCLK_DISABLED_PIN); \
} */
/* #define DBG_PKT_SENT                               \
{                                                     \
    NRF_GPIO->OUTSET = (1 << DBG_PKT_SENT_PIN);       \
    NRF_GPIO->DIRSET = (1 << DBG_PKT_SENT_PIN);       \
    NRF_GPIO->DIRCLR = (1 << DBG_PKT_SENT_PIN);       \
    NRF_GPIO->OUTCLR = (1 << DBG_PKT_SENT_PIN);       \
} */
/* #define DBG_WFE_BEGIN                              \
{                                                     \
    NRF_GPIO->OUTCLR = (1 << DBG_WFE_BEGIN_PIN);      \
    NRF_GPIO->DIRSET = (1 << DBG_WFE_BEGIN_PIN);      \
    NRF_GPIO->OUTSET = (1 << DBG_WFE_BEGIN_PIN);      \
    for ( int i = 0; i < 0xFF; i++ ) __NOP();         \
    NRF_GPIO->OUTCLR = (1 << DBG_WFE_BEGIN_PIN);      \
} */
/* #define DBG_WFE_END                                \
{                                                     \
    for ( int i = 0; i < 0xFF; i++ ) __NOP();         \
    NRF_GPIO->OUTSET = (1 << DBG_WFE_END_PIN);        \
} */


#ifndef DBG_BEACON_START
#define DBG_BEACON_START
#endif
#ifndef DBG_PKT_SENT
#define DBG_PKT_SENT
#endif
#ifndef DBG_HFCLK_ENABLED
#define DBG_HFCLK_ENABLED
#endif
#ifndef DBG_HFCLK_DISABLED
#define DBG_HFCLK_DISABLED
#endif
#ifndef DBG_WFE_BEGIN
#define DBG_WFE_BEGIN
#endif
#ifndef DBG_WFE_END
#define DBG_WFE_END
#endif


#ifdef PCA20014
static const hal_serial_cfg_t serial_cfg =
{
    .twi0.psel.scl = 27,
    .twi0.psel.sda = 26,
};    
#else
#ifdef PCA10036
static const hal_serial_cfg_t serial_cfg =
{
    .twi0.psel.scl = 25,
    .twi0.psel.sda = 23,
};
#else
static const hal_serial_cfg_t serial_cfg =
{
    .twi0.psel.scl = 29,
    .twi0.psel.sda = 25,    
};
#endif  
#endif




#ifdef TEMPERATURE_AND_PRESSURE_BEACON
#define M_BEACON_PDU_TYPE M_BEACON_PDU_TYPE_TEMP_PRES
#endif


#ifdef TEMPERATURE_ONLY_BEACON
#define M_BEACON_PDU_TYPE M_BEACON_PDU_TYPE_TEMP_ONLY
#endif


#define HFCLK_STARTUP_TIME_US                       (1600)              /* The time in microseconds it takes to start up the HF clock*. */
#define INTERVAL_US                                 (100000)           /* The time in microseconds between advertising events. */
#define INITIAL_TIMEOUT                             (INTERVAL_US)       /* The time in microseconds until adverising the first time. */
#define START_OF_INTERVAL_TO_SENSOR_READ_TIME_US    (INTERVAL_US / 2)   /* The time from the start of the latest advertising event until reading the sensor. */
#define SENSOR_SKIP_READ_COUNT                      (1)                /* The number of advertising events between reading the sensor. */


#if INITIAL_TIMEOUT - HFCLK_STARTUP_TIME_US < 400
#error "Initial timeout too short!"
#endif


#define BD_ADDR_OFFS                (3)     /* BLE device address offest of the beacon advertising pdu. */
#define M_BD_ADDR_SIZE              (6)     /* BLE device address size. */

/* Begin - Definitions for beacons with both temperature and pressure. */
#define SINT16_TEMPERATURE_OFFS     (20)    /* The offset of the temperature in the beacon advertising pdu */
#define UINT32_PRESSURE_OFFS        (26)    /* The offset of the pressure in the beacon advertising pdu */
/* End - Definitions for beacons with both temperature and pressure. */

/* Begin - Definitions for beacons with only temperature. */
#define FLOAT32_TEMPERATURE_OFFS    (36)    /* The offset of the temperature in the beacon advertising pdu */
/* End - Definitions for beacons with only temperature. */


/* The beacon types.
 */
typedef enum
{
    M_BEACON_PDU_TYPE_TEMP_ONLY = 0,    ///< Beacon with only temperature data.
    M_BEACON_PDU_TYPE_TEMP_PRES,        ///< Beacon with both temperature and pressure data.
} m_beacon_pdu_type_t;


static bool volatile m_radio_isr_called;    /* Indicates that the radio ISR has executed. */
static bool volatile m_rtc_isr_called;      /* Indicates that the RTC ISR has executed. */
static uint32_t m_time_us;                  /* Keeps track of the latest scheduled point in time. */
static uint32_t m_skip_read_counter = 0;    /* Keeps track on when to read the sensor. */

/* uint8_t m_adv_pdu[]                      = { */
/* 	 0b00100000, // ADV_NONCONN_IND, TxAdd = 0, RxAdd = 0 */
/* 	 0b01001000, // Length = 10 */
/* 	 0xCF,0xCF,0xCF, 0xCF, 0xCF,0xCF, //Address */
/* 	 0xa1,0xa1,0xa1,0xa1 */
/*   }; */

static uint8_t adv_pdu[36 + 3] =
{
    0x42, 0x24, 0x00,
    0xE2, 0xA3, 0x01, 0xE7, 0x61, 0xF7, 0x02, 0x01, 0x04, 0x1A, 0xFF, 0x59, 0x00, 0x02, 0x15, 0x01, 0x12, 0x23,
    0x34, 0x45, 0x56, 0x67, 0x78, 0x89, 0x9A, 0xAB, 0xBC, 0xCD, 0xDE, 0xEF, 0xF0, 0x01, 0x02, 0x03, 0x04, 0xC3
};





/* Waits for the next NVIC event.
 */
#ifdef __GNUC__
static void __INLINE cpu_wfe(void)
#else
static void __forceinline cpu_wfe(void)
#endif
{
    DBG_WFE_BEGIN;
    __WFE();
    __SEV();
    __WFE();
    DBG_WFE_END;
}

/* Sends an advertising PDU on the given channel index.
 */
static void send_one_packet(uint8_t channel_index)
{
    uint8_t i;
    
    m_radio_isr_called = false;
    hal_radio_channel_index_set(channel_index);
    hal_radio_send(adv_pdu);
    while ( !m_radio_isr_called )
    {
        cpu_wfe();
    }
    
    for ( i = 0; i < 9; i++ )
    {
        __NOP();
    }
}


/* Handles beacon managing.
 */
static void beacon_handler(void)
{
    hal_radio_reset();
    hal_timer_start();
    
    m_time_us = INITIAL_TIMEOUT - HFCLK_STARTUP_TIME_US; 

    do
    {


        if ( m_skip_read_counter == 0 )
        {
            m_rtc_isr_called = false;
            hal_timer_timeout_set(m_time_us - START_OF_INTERVAL_TO_SENSOR_READ_TIME_US);
            while ( !m_rtc_isr_called )
            {
                cpu_wfe();
            }
        
            m_rtc_isr_called = false;
            hal_timer_timeout_set(m_time_us - START_OF_INTERVAL_TO_SENSOR_READ_TIME_US + 10000);
            while ( !m_rtc_isr_called )
            {
                cpu_wfe();
            }
            


        }

        m_skip_read_counter = ( (m_skip_read_counter + 1) < SENSOR_SKIP_READ_COUNT ) ? (m_skip_read_counter + 1) : 0;
        
        m_rtc_isr_called = false;
        hal_timer_timeout_set(m_time_us);
        while ( !m_rtc_isr_called )
        {
            cpu_wfe();
        }
        hal_clock_hfclk_enable();
        DBG_HFCLK_ENABLED;

        m_rtc_isr_called = false;
        m_time_us += HFCLK_STARTUP_TIME_US; 
        hal_timer_timeout_set(m_time_us);
        while ( !m_rtc_isr_called )
        {
            cpu_wfe();
        }
		nrf_gpio_pin_toggle(LED2);
        send_one_packet(37);
        DBG_PKT_SENT;
        send_one_packet(38);
        DBG_PKT_SENT;
        send_one_packet(39);
        DBG_PKT_SENT;
        
        hal_clock_hfclk_disable();
        
        DBG_HFCLK_DISABLED;
        
        m_time_us = m_time_us + (INTERVAL_US - HFCLK_STARTUP_TIME_US); 
    } while ( 1 );
}  


int main(void)
{        



	
	DBG_BEACON_START;

    NRF_GPIO->OUTCLR = 0xFFFFFFFF;
    NRF_GPIO->DIRCLR = 0xFFFFFFFF;


	nrf_gpio_cfg_output(LED2);



        
#ifdef DBG_WFE_BEGIN_PIN
    NRF_GPIO->OUTSET = (1 << DBG_WFE_BEGIN_PIN);
    NRF_GPIO->DIRSET = (1 << DBG_WFE_BEGIN_PIN);
#endif
    
    



    
#ifdef DBG_RADIO_ACTIVE_ENABLE
    NRF_GPIOTE->CONFIG[0] = (GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos) 
                          | (DBG_RADIO_ACTIVE_PIN << GPIOTE_CONFIG_PSEL_Pos) 
                          | (GPIOTE_CONFIG_MODE_Task << GPIOTE_CONFIG_MODE_Pos); 

    NRF_PPI->CH[5].TEP = (uint32_t)&(NRF_GPIOTE->TASKS_OUT[0]); 
    NRF_PPI->CH[5].EEP = (uint32_t)&(NRF_RADIO->EVENTS_READY); 
    NRF_PPI->CH[6].TEP = (uint32_t)&(NRF_GPIOTE->TASKS_OUT[0]); 
    NRF_PPI->CH[6].EEP = (uint32_t)&(NRF_RADIO->EVENTS_DISABLED); 

    NRF_PPI->CHENSET = (PPI_CHEN_CH5_Enabled << PPI_CHEN_CH5_Pos)
                     | (PPI_CHEN_CH6_Enabled << PPI_CHEN_CH6_Pos); 
#endif

    for (;;)
    {
        beacon_handler();
    }
}


void RADIO_IRQHandler(void)
{
    NRF_RADIO->EVENTS_DISABLED = 0;
    m_radio_isr_called = true;    
}


void RTC0_IRQHandler(void)
{
    NRF_RTC0->EVTENCLR = (RTC_EVTENCLR_COMPARE0_Enabled << RTC_EVTENCLR_COMPARE0_Pos);
    NRF_RTC0->INTENCLR = (RTC_INTENCLR_COMPARE0_Enabled << RTC_INTENCLR_COMPARE0_Pos);
    NRF_RTC0->EVENTS_COMPARE[0] = 0;
    
    m_rtc_isr_called = true;    
}
