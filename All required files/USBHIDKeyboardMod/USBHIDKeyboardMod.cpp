/*
  Keyboard.cpp

  Copyright (c) 2015, Arduino LLC
  Original code (pre-library): Copyright (c) 2011, Peter Barrett

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
#include "USBHID.h"
#if CONFIG_TINYUSB_HID_ENABLED

#include "USBHIDKeyboardMod.h"

ESP_EVENT_DEFINE_BASE(ARDUINO_USB_HID_KEYBOARD_EVENTS);
esp_err_t arduino_usb_event_post(esp_event_base_t event_base, int32_t event_id, void *event_data, size_t event_data_size, TickType_t ticks_to_wait);
esp_err_t arduino_usb_event_handler_register_with(esp_event_base_t event_base, int32_t event_id, esp_event_handler_t event_handler, void *event_handler_arg);

static const uint8_t report_descriptor[] = {
    TUD_HID_REPORT_DESC_KEYBOARD(HID_REPORT_ID(HID_REPORT_ID_KEYBOARD))};

USBHIDKeyboardMod::USBHIDKeyboardMod() : hid()
{
    // Use US Keyboard as default
    _asciimap = KeyboardLayout_en_US;
    static bool initialized = false;
    if (!initialized)
    {
        initialized = true;
        hid.addDevice(this, sizeof(report_descriptor));
    }
}

uint16_t USBHIDKeyboardMod::_onGetDescriptor(uint8_t *dst)
{
    memcpy(dst, report_descriptor, sizeof(report_descriptor));
    return sizeof(report_descriptor);
}

void USBHIDKeyboardMod::begin(KeyboardLayout *layout)
{
    if (layout != nullptr)
    {
        keyboardlayout = layout;
        _asciimap = layout->getKeymap();
    }
    else
    {
        layout = new KeyboardLayoutUS();
    }
    hid.begin();
}

void USBHIDKeyboardMod::end()
{
}

void USBHIDKeyboardMod::onEvent(esp_event_handler_t callback)
{
    onEvent(ARDUINO_USB_HID_KEYBOARD_ANY_EVENT, callback);
}
void USBHIDKeyboardMod::onEvent(arduino_usb_hid_keyboard_event_t event, esp_event_handler_t callback)
{
    arduino_usb_event_handler_register_with(ARDUINO_USB_HID_KEYBOARD_EVENTS, event, callback, this);
}

void USBHIDKeyboardMod::_onOutput(uint8_t report_id, const uint8_t *buffer, uint16_t len)
{
    if (report_id == HID_REPORT_ID_KEYBOARD)
    {
        arduino_usb_hid_keyboard_event_data_t p;
        p.leds = buffer[0];
        arduino_usb_event_post(ARDUINO_USB_HID_KEYBOARD_EVENTS, ARDUINO_USB_HID_KEYBOARD_LED_EVENT, &p, sizeof(arduino_usb_hid_keyboard_event_data_t), portMAX_DELAY);
    }
}

void USBHIDKeyboardMod::sendReport(KeyReport *keys)
{
    hid_keyboard_report_t report;
    report.reserved = 0;
    report.modifier = keys->modifiers;
    if (keys->keys)
    {
        memcpy(report.keycode, keys->keys, 6);
    }
    else
    {
        memset(report.keycode, 0, 6);
    }
    hid.SendReport(HID_REPORT_ID_KEYBOARD, &report, sizeof(report));
}

size_t USBHIDKeyboardMod::pressRaw(uint8_t k)
{
    uint8_t i;
    if (k >= 0xE0 && k < 0xE8)
    {
        // it's a modifier key
        _keyReport.modifiers |= (1 << (k - 0x80));
    }
    else if (k && k < 0xA5)
    {
        // Add k to the key report only if it's not already present
        // and if there is an empty slot.
        if (_keyReport.keys[0] != k && _keyReport.keys[1] != k &&
            _keyReport.keys[2] != k && _keyReport.keys[3] != k &&
            _keyReport.keys[4] != k && _keyReport.keys[5] != k)
        {

            for (i = 0; i < 6; i++)
            {
                if (_keyReport.keys[i] == 0x00)
                {
                    _keyReport.keys[i] = k;
                    break;
                }
            }
            if (i == 6)
            {
                return 0;
            }
        }
    }
    else
    {
        // not a modifier and not a key
        return 0;
    }
    sendReport(&_keyReport);
    return 1;
}

size_t USBHIDKeyboardMod::releaseRaw(uint8_t k)
{
    uint8_t i;
    if (k >= 0xE0 && k < 0xE8)
    {
        // it's a modifier key
        _keyReport.modifiers &= ~(1 << (k - 0x80));
    }
    else if (k && k < 0xA5)
    {
        // Test the key report to see if k is present.  Clear it if it exists.
        // Check all positions in case the key is present more than once (which it shouldn't be)
        for (i = 0; i < 6; i++)
        {
            if (0 != k && _keyReport.keys[i] == k)
            {
                _keyReport.keys[i] = 0x00;
            }
        }
    }
    else
    {
        // not a modifier and not a key
        return 0;
    }

    sendReport(&_keyReport);
    return 1;
}

void USBHIDKeyboardMod::setModifier(uint8_t modifier)
{
    _keyReport.modifiers |= modifier;
    sendReport(&_keyReport);
}

void USBHIDKeyboardMod::unsetModifier(uint8_t modifier)
{
    _keyReport.modifiers &= ~(modifier);
    sendReport(&_keyReport);
}

// press() adds the specified key (printing, non-printing, or modifier)
// to the persistent key report and sends the report.  Because of the way
// USB HID works, the host acts like the key remains pressed until we
// call release(), releaseAll(), or otherwise clear the report and resend.
size_t USBHIDKeyboardMod::press(uint8_t k)
{

    if (k >= 0x88)
    { // it's a non-printing key (not a modifier)
        k = k - 0x88;
    }
    else if (k >= 0x80)
    { // it's a modifier key
        _keyReport.modifiers |= (1 << (k - 0x80));
        k = 0;
    }
    else
    { // it's a printing key

        k = _asciimap[k];
        if (!k)
        {
            return 0;
        }
        if ((k & ALT_GR) == ALT_GR)
        {
            _keyReport.modifiers |= 0x40; // AltGr = right Alt
            k &= 0x3F;
        }
        else if ((k & SHIFT) == SHIFT)
        {                                 // it's a capital letter or other character reached with shift
            _keyReport.modifiers |= 0x02; // the left shift modifier
            k &= 0x7F;
        }
        if (k == ISO_REPLACEMENT)
        {
            k = ISO_KEY;
        }
    }
    return pressRaw(k);
}

// release() takes the specified key out of the persistent key report and
// sends the report.  This tells the OS the key is no longer pressed and that
// it shouldn't be repeated any more.
size_t USBHIDKeyboardMod::release(uint8_t k)
{
    if (k >= 0x88)
    { // it's a non-printing key (not a modifier)
        Serial.println("No printing key");
        Serial.println((char)k);
        k = k - 0x88;
    }
    else if (k >= 0x80)
    { // it's a modifier key
        _keyReport.modifiers &= ~(1 << (k - 0x80));
        k = 0;
    }
    else
    { // it's a printing key

        k = _asciimap[k];

        if (!k)
        {
            return 0;
        }
        if ((k & ALT_GR) == ALT_GR)
        {
            _keyReport.modifiers &= ~(0x40); // AltGr = right Alt
            k &= 0x3F;
        }
        else if ((k & SHIFT) == SHIFT)
        {
            _keyReport.modifiers &= ~(0x02); // the left shift modifier
            k &= 0x7F;
        }
        if (k == ISO_REPLACEMENT)
        {
            k = ISO_KEY;
        }
    }
    return releaseRaw(k);
}

void USBHIDKeyboardMod::releaseAll(void)
{
    _keyReport.keys[0] = 0;
    _keyReport.keys[1] = 0;
    _keyReport.keys[2] = 0;
    _keyReport.keys[3] = 0;
    _keyReport.keys[4] = 0;
    _keyReport.keys[5] = 0;
    _keyReport.modifiers = 0;
    sendReport(&_keyReport);
}

size_t USBHIDKeyboardMod::write(uint8_t c)
{
    uint8_t p = press(c); // Keydown
    release(c);           // Keyup
    return p;             // just return the result of press() since release() almost always returns 1
}

size_t USBHIDKeyboardMod::write(const uint8_t *buffer, size_t size)
{
    size_t n = 0;
    while (size--)
    {
        if (*buffer != '\r')
        {
            if (write(*buffer))
            {
                n++;
            }
            else
            {
                break;
            }
        }
        buffer++;
    }
    return n;
}

size_t USBHIDKeyboardMod::press(uint16_t key)
{
    if (key > 0x88)
    {
        uint16_t keycode = keyboardlayout->isSpecialKey(key);
        if (keycode & 0x8000)
        {
            setModifier(0x02);
            keycode = keycode & 0x7FFF;
            return press((uint8_t)keycode);
        }
        else
        {
            return press((uint8_t)keycode);
        }
    }
    else
    {
        return press((uint8_t)key);
    }
}
size_t USBHIDKeyboardMod::release(uint16_t key)
{
    if (key > 0x88)
    {
        uint16_t keycode = keyboardlayout->isSpecialKey(key);
        if (keycode & 0x8000)
        {
            keycode = keycode & 0x7FFF;
            auto returnval = release((uint8_t)keycode);
            unsetModifier(0x02);
            return returnval;
        }
        else
        {
            return release((uint8_t)keycode);
        }
    }
    else
    {
        return release((uint8_t)key);
    }
}

size_t USBHIDKeyboardMod::write(uint16_t k)
{
    uint8_t p = press(k); // Keydown
    release(k);           // Keyup
    return p;             // just return the result of press() since release() almost always returns 1
}
size_t USBHIDKeyboardMod::write(const uint16_t *buffer, size_t size)
{
    size_t n = 0;
    while (size--)
    {
        if (*buffer != '\r')
        {
            if (write(*buffer))
            {
                n++;
            }
            else
            {
                break;
            }
        }
        buffer++;
    }
    return n;
}
size_t USBHIDKeyboardMod::write(std::u16string text)
{
    for (uint16_t i = 0; i < text.length(); i++)
    {
        press((uint16_t)text.at(i));
        release((uint16_t)text.at(i));
    }
    return text.length();
}

#endif /* CONFIG_TINYUSB_HID_ENABLED */
