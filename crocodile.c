/*************************************************************************
 * main.c - Main firmware                                                *
 * Version 0.98                                                          *
 * $Id:: main.c 26 2012-01-22 18:27:08Z spacemanspiff                  $ *
 *************************************************************************
 * SpiffChorder - Spaceman Spiff's Chording Keyboard Experiment          *
 * Copyright (C) 2006-2012  Mikkel Holm Olsen                            *
 *                                                                       *
 * This program is free software: you can redistribute it and/or modify  *
 * it under the terms of the GNU General Public License as published by  *
 * the Free Software Foundation, either version 3 of the License, or     *
 * (at your option) any later version.                                   *
 *                                                                       *
 * This program is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          *
 * GNU General Public License for more details.                          *
 *                                                                       *
 * You should have received a copy of the GNU General Public License     *
 * along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 *************************************************************************/

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <string.h>

#include "keymaps/nasa_colemak.h"

#include "crocodile.h"

/* Hardware documentation:
 *
 * PB1         : Pinky key
 * PD0         : Ring finger key
 * PD4         : Middle finger key
 * PC6         : Index finger key
 * PB5         : Near thumb
 * PE6         : Center thumb
 * PD7         : Far thumb
 *
 */

/* The repeat state and counter in key_repeat can be one of these values
     to signify the state, or it can be higher than REP_STATE_TIMEOUT to
     signify that the timer is running but time-out has not yet occurred */
enum repeat_state_e {
    REP_STATE_READY = 0,
    REP_STATE_KEYDOWN_SENT,
    REP_STATE_TIMEOUT
};

/* Key repeat timer/counter. if>REP_STATE_TIMEOUT then decreased by main loop */
volatile uint8_t key_repeat = REP_STATE_READY;

uint8_t repeat_value = REP_STATE_TIMEOUT + (int)((double)F_CPU / 262144 / 2 + 0.5); // ~ 500ms timeout
uint8_t debounce_value = 10;  // 10 ms debounce. Should be loaded from EEPROM.
uint8_t use_early_detect = 1; // This is a configuration option

/* Originally used as a mask for the modifier bits, but now also
     used for other x -> 2^x conversions (lookup table). */
const uint8_t modmask[8] PROGMEM = {
    0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80
};

/* An array of pointers to the decoding tables */
const prog_keymap_t * const keymap_tables[] PROGMEM = {
    keymap_default,                                    // The default mode
    keymap_numsym,                                     // Numbers/Symbols mode
    keymap_function,                                   // Function key mode
    keymap_cyrillic                                    // Cyrillic key mode
};

const prog_keymap_t *current_keymap = keymap_default;
const prog_keymap_t *special_keymap = NULL;

static uint8_t report_key = 0;
static uint8_t report_mods = 0;

/* Debounce counter. if>1 then decreased by timer interrupt */
volatile uint8_t debounce = 0;

/** Buffer to hold the previously generated Keyboard HID report, for comparison purposes inside the HID class driver. */
static uint8_t PrevKeyboardHIDReportBuffer[sizeof(USB_KeyboardReport_Data_t)];

/** LUFA HID Class driver interface configuration and state information. This structure is
 *  passed to all HID Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
USB_ClassInfo_HID_Device_t Keyboard_HID_Interface =
	{
		.Config =
			{
				.InterfaceNumber              = INTERFACE_ID_Keyboard,
				.ReportINEndpoint             =
					{
						.Address              = KEYBOARD_EPADDR,
						.Size                 = KEYBOARD_EPSIZE,
						.Banks                = 1,
					},
				.PrevReportINBuffer           = PrevKeyboardHIDReportBuffer,
				.PrevReportINBufferSize       = sizeof(PrevKeyboardHIDReportBuffer),
			},
	};


static void hardwareInit(void)
{
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();
    PORTE = 0x40;   /* Enable pull-ups */
    DDRE  = 0x00;   /* Port E are switch inputs */
    PORTD = 0x91;   /* Enable pull-ups */
    DDRD  = 0x00;   /* Port D are switch inputs */
    PORTC = 0x40;   /* Enable pull-ups */
    DDRC  = 0x00;   /* Port C are switch inputs */
    PORTB = 0x22;   /* Enable pull-ups */
    DDRB  = 0x00;   /* Port B are switch inputs */

    PORTE = 0xff;   /* Enable pull-ups */
    DDRE  = 0x00;   /* Port E are switch inputs */
    PORTD = 0xff;   /* Enable pull-ups */
    DDRD  = 0x00;   /* Port D are switch inputs */
    PORTC = 0xff;   /* Enable pull-ups */
    DDRC  = 0x00;   /* Port C are switch inputs */
    PORTB = 0xff;   /* Enable pull-ups */
    DDRB  = 0x00;   /* Port B are switch inputs */
	/* Disable clock division */
	clock_prescale_set(clock_div_1);

	/* Hardware Initialization */
	USB_Init();

    /* configure timer 0 for a rate of 16M/(1024 * 256) = 61.78 Hz (~16ms) */
    TCCR0A = 0;              /* Normal Mode, OC disabled */
    TCCR0B = (5 << CS00);    /* timer 0 prescaler: 1024 */
    /* configure timer 2 for a rate of 16M/(256 * 64) = 992.0 Hz (~1ms) */
    OCR1A = (uint16_t)((double) F_CPU / 256 / 1000 + 0.5);
    TCCR1A = 0;
    TCCR1B = (1 << WGM12) | (4 << CS10);    /* prescaler=256, CTC-mode */
    TIMSK1 |= (1 << OCIE1A); /* Enable compare match interrupt */
}

/* Timer 2 interrupt handler - decrease debounce counter every 1ms */
ISR(TIMER1_COMPA_vect)
{
    /* This gets called every 1ms and updates the button debounce counter */
    if (debounce>1) 
    {
        --debounce;
    }
}

/* The modifier keys applied to next keypress and the modifier locks */
uint8_t mod_state=0, mod_lock=0;

/* The current mode and whether to lock the mode or only apply it to one keypress */
uint8_t base_mode=0, current_mode=0, lock_mode=0, last_modifiers=0;

/* Used to send the key_up event */
const uint8_t PROGMEM key_up[] = {0};

/* This pointer tells us something needs to be sent before returning to keyboard scanning */
const uint8_t *pendingPtr = key_up;

/* Handle a chord */
bool process_chord(uint8_t chord)
{
    uint8_t key;
#ifdef KEYMAP_USES_MODIFIERS
    keymap_t keymods;
    keymods = pgm_read_word(special_keymap?&special_keymap[chord&0x7F]:
                                        &current_keymap[chord&0x7F]);
    key = (uint8_t)keymods; /* typecast masks off upper bits */
#else
    key = pgm_read_byte(special_keymap?&special_keymap[chord&0x7F]:
                                        &current_keymap[chord&0x7F]);
#endif
    special_keymap = NULL;
    if (key >= DIV_Mods)
    { // This is not a standard key
        if (key >= DIV_Multi)
        { // Either Multi-mode or Macro
            if (key >= DIV_Macro)
            { /* Macro */
                pendingPtr = (const uint8_t*)pgm_read_word(&macro_strings[key-DIV_Macro]);
                mod_state &= mod_lock; 
                if (!lock_mode)
                { // Reset mode unless it was locked
                    current_mode = base_mode;
                    current_keymap = (const prog_keymap_t*)pgm_read_word(&keymap_tables[current_mode]);
                }
            }
            else
            { // Multi
                special_keymap = keymap_numsym; // Numbers/symbols-mode
                mod_state |= 0x02; // Left shift
            }
        }
        else
        { // Either mods or modes
            if (key >= DIV_Modes)
            { /* Mode change requested */
                if (key == MODE_RETURN)
                {
                    current_mode = base_mode;
                    lock_mode = 0;
                }
                else if ((key == MODE_RU) || (key == MODE_RESET))
                {
                    base_mode = (key - DIV_Modes) >> 1;
                    current_mode = base_mode;
                    lock_mode = 0;
                }
                else
                {
                    current_mode = (key-DIV_Modes);
                    lock_mode = current_mode & 0x01;
                    current_mode >>= 1;
                }
                current_keymap = (const prog_keymap_t*)pgm_read_word(&keymap_tables[current_mode]);
            }
            else
            { // Modifier
                mod_state |= pgm_read_byte(&modmask[key-DIV_Mods]);
            }
        }
        key = 0;
    }

    if (key)
    {
        report_mods = 0;
        report_key = key;

#ifdef KEYMAP_USES_MODIFIERS
        key = keymods >> 8;
        if (key)
        {
            report_mods = key;
        }
        else 
#endif
        {
            report_mods = mod_state;
        }
        last_modifiers = report_mods; /* Save the modifiers for proper clean-up on release */

        mod_state &= mod_lock;
        if (!lock_mode)
        {
            current_mode = base_mode;
            current_keymap = (const prog_keymap_t*)pgm_read_word(&keymap_tables[current_mode]);
        }
        return true;
    }
    return false;
}

static uint8_t latched=0xFF, adding=0;

/* When the state of keys change, handle key-down or key-up */
bool key_change(uint8_t data, bool timeout)
{
    bool retval = false;
    if (timeout)
    {
        // If the same chord has been held for an amount of time - send key-down - repeat starts
        if (adding)
            retval = process_chord(~latched);
    }
    else
    {
        if (latched & ~data)
        { // buttons(s) pressed
            adding = 1;
            latched &= data;
            key_repeat = repeat_value;
        }
        else if (~latched & data)
        { // buttons(s) released
            if (use_early_detect ? adding : data == 0xFF)
            {
                if (key_repeat != REP_STATE_KEYDOWN_SENT)
                    retval = process_chord(~latched);
                latched=data;
                adding=0;
            }
            if ((retval) || (key_repeat == REP_STATE_KEYDOWN_SENT))
            { // Some key was reported
                if (!pendingPtr)
                    pendingPtr = key_up; // Make sure the key up event is sent immediately
            }
            key_repeat = REP_STATE_READY;
            if (use_early_detect)
                latched = data;
        }
    }
    return retval;
}

static uint8_t olddata = 0xFF;

/* Scan the keys, debounce, and report changes in state */
bool scankeys(void)
{
    bool retval = false;
    uint8_t data;

    data = ((PINB & 0x02) >> 1) << 0 |
           ((PIND & 0x01) >> 0) << 1 |
           ((PIND & 0x10) >> 4) << 2 |
           ((PINC & 0x40) >> 6) << 3 |
           ((PINB & 0x20) >> 5) << 4 |
           ((PINE & 0x40) >> 6) << 5 |
           ((PIND & 0x80) >> 7) << 6;

    if (olddata ^ data)
    { // Change detected
        if (debounce == 0)
        {
            debounce = debounce_value;
        }
        else if (debounce == 1)
        {
            retval = key_change(data, false); // Process key change
            olddata = data;
            debounce = 0;
        }
    }
    else
    {
        if (key_repeat == REP_STATE_TIMEOUT)
        {
            retval = key_change(data, true); // Send key-down
            key_repeat = REP_STATE_KEYDOWN_SENT;
        }
    }

    return retval;
}

static uint8_t state = 0;

bool getCharFromPending(void)
{
    uint8_t kval = 0;
    report_key = 0;
    report_mods = 0;
    if (state)
    {
        report_mods = last_modifiers;
        state = 0;
    }
    else
    {
        kval = pgm_read_byte(pendingPtr);
        report_mods = (kval & 0x80 ? 0x02 : 0);
        report_key = kval & ~0x80;
        if (kval)
            pendingPtr++;
        else
            pendingPtr = NULL;
        state = 1;
    }
    return true;
}

bool updateNeeded = false;

uint8_t i = 0;

int main(void)
{
    wdt_enable(WDTO_2S); /* Enable watchdog timer 2s */
    hardwareInit(); /* Initialize hardware (I/O) */
    
	GlobalInterruptEnable();
    
    for(;;)
    {    /* Main loop */
        wdt_reset(); /* Reset the watchdog */
        
        if (!updateNeeded)
        {
            if (pendingPtr)
            {
                updateNeeded = getCharFromPending(); /* Keys pending in output buffer? */
            }
            else
            {
                updateNeeded = scankeys(); /* Scan the keyboard for changes */
            }
        }

        /* Check timer if we need periodic reports */
        if (TIFR0 & (1 << TOV0))
        {
            TIFR0 = 1 << TOV0; /* Reset flag */
            if (key_repeat > REP_STATE_TIMEOUT)
                --key_repeat;
/*            i++;
            if (i > 100)
            {
                i = 0;
                updateNeeded = true;
            }*/
        }
		HID_Device_USBTask(&Keyboard_HID_Interface);
		USB_USBTask();
    }
    return 0;
}

/** Event handler for the library USB Connection event. */
void EVENT_USB_Device_Connect(void)
{
}

/** Event handler for the library USB Disconnection event. */
void EVENT_USB_Device_Disconnect(void)
{
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	bool ConfigSuccess = true;

	ConfigSuccess &= HID_Device_ConfigureEndpoints(&Keyboard_HID_Interface);

	USB_Device_EnableSOFEvents();
}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void)
{
	HID_Device_ProcessControlRequest(&Keyboard_HID_Interface);
}

/** Event handler for the USB device Start Of Frame event. */
void EVENT_USB_Device_StartOfFrame(void)
{
	HID_Device_MillisecondElapsed(&Keyboard_HID_Interface);
}

/** HID class driver callback function for the creation of HID reports to the host.
 *
 *  \param[in]     HIDInterfaceInfo  Pointer to the HID class interface configuration structure being referenced
 *  \param[in,out] ReportID    Report ID requested by the host if non-zero, otherwise callback should set to the generated report ID
 *  \param[in]     ReportType  Type of the report to create, either HID_REPORT_ITEM_In or HID_REPORT_ITEM_Feature
 *  \param[out]    ReportData  Pointer to a buffer where the created report should be stored
 *  \param[out]    ReportSize  Number of bytes written in the report (or zero if no report is to be sent)
 *
 *  \return Boolean \c true to force the sending of the report, \c false to let the library determine if it needs to be sent
 */
bool CALLBACK_HID_Device_CreateHIDReport(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo,
                                         uint8_t* const ReportID,
                                         const uint8_t ReportType,
                                         void* ReportData,
                                         uint16_t* const ReportSize)
{
	USB_KeyboardReport_Data_t* KeyboardReport = (USB_KeyboardReport_Data_t*)ReportData;

    KeyboardReport->KeyCode[0] = report_key;
    KeyboardReport->Modifier = report_mods;
    if ((current_mode == ((MODE_RU-DIV_Modes) >> 1)) && report_key)
       KeyboardReport->Modifier |= pgm_read_byte(&modmask[MOD_CYR]);
            
    updateNeeded = false;

  	*ReportSize = sizeof(USB_KeyboardReport_Data_t);

	return false;
}

/** HID class driver callback function for the processing of HID reports from the host.
 *
 *  \param[in] HIDInterfaceInfo  Pointer to the HID class interface configuration structure being referenced
 *  \param[in] ReportID    Report ID of the received report from the host
 *  \param[in] ReportType  The type of report that the host has sent, either HID_REPORT_ITEM_Out or HID_REPORT_ITEM_Feature
 *  \param[in] ReportData  Pointer to a buffer where the received report has been stored
 *  \param[in] ReportSize  Size in bytes of the received HID report
 */
void CALLBACK_HID_Device_ProcessHIDReport(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo,
                                          const uint8_t ReportID,
                                          const uint8_t ReportType,
                                          const void* ReportData,
                                          const uint16_t ReportSize)
{
}

