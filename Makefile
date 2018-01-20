all: broadcast-example unicast-sender unicast-receiver
APPS = servreg-hack serial-ubidots
CONTIKI=/home/user/contiki

CONTIKI_WITH_IPV6 = 1
include $(CONTIKI)/Makefile.include
