#include <gui/common/FrontendApplication.hpp>
#ifdef TARGET
#include ".\flash\bsp_spi_flash.h"
#endif


//#define USE_BINARY_FONT
#ifdef USE_BINARY_FONT
uint8_t fontData[10240];
touchgfx::BinaryFont bf;
extern const uint8_t unicodes_verdana_40_4bpp_0[];

#endif

FrontendApplication::FrontendApplication(Model& m, FrontendHeap& heap)
    : FrontendApplicationBase(m, heap)
{
#ifdef USE_BINARY_FONT
    SpiReadFlash(fontData, unicodes_verdana_40_4bpp_0, 10240);
    new ($bf) BinaryFont((const struct touchgfx::BinaryFontData* )fontData);
    TypedTextDatabase::setFont(DEFAULT, &bf);
#endif
}
