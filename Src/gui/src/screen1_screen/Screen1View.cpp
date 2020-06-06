#include <gui/screen1_screen/Screen1View.hpp>

Screen1View::Screen1View()
{

}

void Screen1View::setupScreen()
{
#ifdef TARGET
    Bitmap::cacheAll();
#endif
    Screen1ViewBase::setupScreen();
}

void Screen1View::tearDownScreen()
{
#ifdef TARGET
    Bitmap::clearCache();
#endif
    Screen1ViewBase::tearDownScreen();
}
