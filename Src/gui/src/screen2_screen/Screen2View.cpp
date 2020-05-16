#include <gui/screen2_screen/Screen2View.hpp>

Screen2View::Screen2View()
{

}

void Screen2View::setupScreen()
{
    Screen2ViewBase::setupScreen();
    analogHours = analogClock1.getCurrentHour();
    analogMinutes = analogClock1.getCurrentMinute();
    analogSeconds = analogClock1.getCurrentSecond();

    digitalHours = digitalClock1.getCurrentHour();
    digitalMinutes = digitalClock1.getCurrentMinute();
    digitalSeconds = digitalClock1.getCurrentSecond();
}

void Screen2View::tearDownScreen()
{
    Screen2ViewBase::tearDownScreen();
}

void Screen2View::handleTickEvent()
{
    tickCounter++;

    if (tickCounter % 60 == 0) {
        if (++analogSeconds >= 60) {
            analogSeconds = 0;
            if (++analogMinutes >= 60) {
                analogMinutes = 0;
                if (++analogHours >= 24) {
                    analogHours = 0;
                }
            }
        }
        analogClock1.setTime24Hour(analogHours, analogMinutes, analogSeconds);

        if (++digitalSeconds >= 60) {
            digitalSeconds = 0;
            if (++digitalMinutes >= 60) {
                digitalMinutes = 0;
                if (++digitalHours >= 24) {
                    digitalHours = 0;
                }
            }
        }
        digitalClock1.setTime24Hour(digitalHours, digitalMinutes, digitalSeconds);
    }
}
