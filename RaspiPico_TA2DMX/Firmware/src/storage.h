#ifndef STORAGE_H
#define STORAGE_H

#include "config.h"

// Flash'ın son 4KB'lık sektörünü kullanacağız
void storage_init();
void save_channel(ChannelConfig *cfg);
void load_channel(ChannelConfig *cfg);

#endif
