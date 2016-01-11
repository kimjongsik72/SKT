DEFINES+=PROJECT_CONF_H=\"project-conf.h\"
DEFINES+=COMMAND_H=\"command.h\"
DEFINES+=GPIO_RELAY_H=\"gpiorelay.h\"
CONTIKI_PROJECT = SKT-Current-brodcast-demo

all: $(CONTIKI_PROJECT)


PROJECT_SOURCEFILES += gpiorelay.c 

CONTIKI_WITH_RIME = 1

CONTIKI=../../..
include $(CONTIKI)/Makefile.include
