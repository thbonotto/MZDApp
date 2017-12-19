COMPONENT=TestGTSAppC
CFLAGS += -I$(TOSDIR)/lib/printf
CFLAGS += 
CFLAGS += 
CFLAGS += -I$(shell pwd)/..

CFLAGS += -DIEEE154_GTS_DEVICE_ENABLED  -DIEEE154_BEACON_TX_DISABLED  -DIEEE154_PROMISCUOUS_MODE_DISABLED -DTKN154_ACTIVE_MESSAGE_SUPPORT_DISABLED

# To use the TKN15.4 MAC instead of a platform's default MAC protocol first
# include the TinyOS "Makerules" file as usual ...
include $(TINYOS_ROOT_DIR)/Makefile.include

# ... and then include the TKN15.4 "Makefile.include" file. That's all.
# Hint: type "make <platform> verbose" to see the aggregate include path.
include $(TOSDIR)/lib/mac/tkn154/Makefile.include
