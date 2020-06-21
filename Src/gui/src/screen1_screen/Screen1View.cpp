#include <gui/screen1_screen/Screen1View.hpp>
#include "fonts/FontCache.hpp"
#include "fonts/CachedFont.hpp"
#include "touchgfx/TypedText.hpp"
#include "texts/TextKeysAndLanguages.hpp"
#include "fonts/ApplicationFontProvider.hpp"
#include "texts/TypedTextDatabase.hpp"
#include ".\flash\bsp_spi_flash.h"


#ifdef USE_FONT_CACHE
class FileDataReader : public FontDataReader
{
public:
    virtual ~FileDataReader() {}
    virtual void open() {}
    virtual void close() {}
    virtual void setPosition(uint32_t position)
    {
        fontAddr = position;
    }
    virtual void readData(void* out, uint32_t numberOfBytes)
    {
        SpiReadFlash((uint8_t* )out, fontAddr, numberOfBytes);
    }

private:
    uint32_t fontAddr;
};

uint8_t fontData[5120];
touchgfx::FontCache fontCache;
touchgfx::CachedFont cachedFont;
FileDataReader reader;
#endif

Screen1View::Screen1View()
{
}

void Screen1View::setupScreen()
{
#ifdef TARGET
    Bitmap::cacheAll();
#endif
#ifdef USE_FONT_CACHE
    fontCache.setMemory(fontData, 5120);
    fontCache.setReader(&reader);
    touchgfx::TypedText text = touchgfx::TypedText(T_SINGLEUSEID1);
    fontCache.initializeCachedFont(text, &cachedFont);
    TypedTextDatabase::setFont(Typography::DEFAULT, &cachedFont);
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
