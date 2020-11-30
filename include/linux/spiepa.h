#ifndef __SPIEPA_H__
#define __SPIEPA_H__
#include <uapi/linux/spiepa.h>

#define SPIEPA_EMR_WIDTH_PROPERTY	"ratta,emr-width"
#define SPIEPA_EMR_HEIGHT_PROPERTY	"ratta,emr-height"
#define SPIEPA_EPD_WIDTH_PROPERTY	"ratta,epd-width"
#define SPIEPA_EPD_HEIGHT_PROPERTY	"ratta,epd-height"

#define SPIEPA_TOOL_PEN		((u8)0)
#define SPIEPA_TOOL_RUBBER	((u8)1)

extern void spiepa_event_enqueue(int x, int y, int p, u8, u8);

#endif
