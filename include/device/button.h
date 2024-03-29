#ifndef OKAPI_BUTTON
#define OKAPI_BUTTON

#include "PAL/PAL.h"

namespace okapi {
  class Button {
  public:
    explicit constexpr Button(const unsigned char iport, const bool iinverted = false):
      joystick(1),
      buttonGroup(8),
      port(iport),
      lcd(uart1),
      inverted(iinverted),
      isJoystick(false),
      isLCD(false),
      wasPressedLast(iinverted) {}

    explicit constexpr Button(const unsigned char ijoystick, const unsigned char ibuttonGroup, const unsigned char ibutton, const bool iinverted = false):
      joystick(ijoystick),
      buttonGroup(ibuttonGroup),
      port(ibutton),
      lcd(uart1),
      inverted(iinverted),
      isJoystick(true),
      isLCD(false),
      wasPressedLast(iinverted) {}

    explicit constexpr Button(PROS_FILE* ilcdPort, const unsigned char ilcdButton, const bool iinverted = false):
      joystick(1),
      buttonGroup(8),
      port(ilcdButton),
      lcd(ilcdPort),
      inverted(iinverted),
      isJoystick(false),
      isLCD(true),
      wasPressedLast(iinverted) {}

    bool isPressed() const {
      if (isJoystick)
        return inverted ? !PAL::joystickGetDigital(joystick, buttonGroup, port) : PAL::joystickGetDigital(joystick, buttonGroup, port);
      else if (isLCD)
        return inverted ? !(PAL::lcdReadButtons(lcd) == port) : (PAL::lcdReadButtons(lcd) == port);
      else
        return inverted ? !PAL::digitalRead(port) : PAL::digitalRead(port);
    }

    bool edge() {
      const bool pressed = isPressed();
      const bool out = pressed ^ wasPressedLast;
      wasPressedLast = pressed;
      return out;
    }

    bool risingEdge() { return edge() && wasPressedLast; } //Remember edge sets wasPressedLast

    bool fallingEdge() { return edge() && !wasPressedLast; }  //Remember edge sets wasPressedLast
  private:
    const unsigned char joystick, buttonGroup, port;
    PROS_FILE *lcd;
    const bool inverted, isJoystick, isLCD;
    bool wasPressedLast;
  };

  inline namespace literals {
    constexpr Button operator"" _b(const unsigned long long int p) { return Button(static_cast<unsigned char>(p), false); }
    constexpr Button operator"" _ib(const unsigned long long int p) { return Button(static_cast<unsigned char>(p), true); }
  }
}

#endif /* end of include guard: OKAPI_BUTTON */
