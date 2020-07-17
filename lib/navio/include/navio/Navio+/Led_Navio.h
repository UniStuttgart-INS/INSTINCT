#ifndef LED_NAVIO_H
#define LED_NAVIO_H

#include <navio/Common/Led.h>
#include "navio/Navio+/PCA9685.h"
#include <navio/Common/gpio.h>

using namespace Navio;

class Led_Navio : public Led
{
  public:
    Led_Navio();
    bool initialize() override;
    void setColor(Colors color) override;

  private:
    PCA9685 led;
};

#endif // LED_NAVIO_H
