NECRO_CONFIG := $(PKG_CONFIG_SYSROOT_DIR)/usr/necro/bin/necro-config
CPPFLAGS +=  $(shell DESTDIR=$(PKG_CONFIG_SYSROOT_DIR) $(NECRO_CONFIG) --qiuniu --cflags)
LDFLAGS += $(shell DESTDIR=$(PKG_CONFIG_SYSROOT_DIR) $(NECRO_CONFIG) --qiuniu --rldf --ldflags --no-auto-init)

# INCLUDES = -I$(PKG_CONFIG_SYSROOT_DIR)/usr/include/

LIBS = -lethercat-cpp -L$(PKG_CONFIG_SYSROOT_DIR)/usr/lib/

all: ethercat-c-demo

ethercat-c-demo: main.o spi_io.o create_threads.o
	$(CC) $(CPPFLAGS) $(LDFLAGS) -o $@ $^ $(INCLUDES) $(LIBS)
	$(RM) *.o